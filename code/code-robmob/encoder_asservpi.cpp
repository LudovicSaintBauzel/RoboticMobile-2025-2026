// encoder_asservpi.cpp — Encodeurs quadrature + calcul vitesse + PID moteur I2C
// Standard : C++20  (jthread, atomic, chrono, stop_token)
//
// ── Compilation mode virtuel (macOS / test sans GPIO) ──────────────────────
//   g++ -std=c++20 -O2 -Wall encoder_asservpi.cpp -o encoder_asservpi
//
// ── Compilation mode Raspberry Pi ──────────────────────────────────────────
//   sudo apt install wiringpi   (ou compiler depuis le source sur Pi 4+)
//   g++ -std=c++20 -O2 -Wall -DRPI encoder_asservpi.cpp -o encoder_asservpi 
//       -lwiringPi -lpthread
//
// GPIO matériel géré via wiringPi (ISR sur fronts BOTH, numérotation BCM).
// I2C via l'API POSIX Linux (open/ioctl/write sur /dev/i2c-1).
//
// ── Usage ────────────────────────────────────────────────────────────────────
//   ./encoder_asservpi              → M1=M2=80 RPM (avant)
//   ./encoder_asservpi 60           → M1=M2=60 RPM (avant)
//   ./encoder_asservpi 80 -60       → M1=80 RPM avant, M2=60 RPM arrière
//   RPM positif = avant (CW), RPM négatif = arrière (CCW)

#include <algorithm>   // std::max, std::min, std::clamp
#include <atomic>
#include <chrono>
#include <cmath>       // std::abs, std::round
#include <csignal>
#include <cstdint>
#include <cstring>     // std::memcpy
#include <iomanip>     // std::fixed, std::setprecision
#include <iostream>
#include <memory>
#include <stdexcept>   // std::invalid_argument
#include <string>      // std::stod
#include <thread>

#ifdef RPI
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#  include <linux/i2c-dev.h>
#  include <wiringPi.h>
#endif

using namespace std;

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

// Broches GPIO (BCM) — utilisées uniquement en mode RPI
[[maybe_unused]] constexpr uint8_t  ENC1_A        = 19;
[[maybe_unused]] constexpr uint8_t  ENC1_B        = 18;
[[maybe_unused]] constexpr uint8_t  ENC2_A        = 17;
[[maybe_unused]] constexpr uint8_t  ENC2_B        = 16;

constexpr double   ENCODER_CPR   = 1440.0;

// Paramètres I2C / PID — utilisés uniquement en mode RPI
[[maybe_unused]] constexpr int      I2C_ADDR      = 0x0F;
[[maybe_unused]] constexpr uint32_t PID_PERIOD_MS = 50;

// Registres Grove I2C Motor Driver V1.3 (L298P)
// 0x82 : vitesse dual moteur [cmd_m1, cmd_m2] (magnitude 0-255)
// 0xAA : direction dual moteur [dir_byte, 0x01]
//   dir_byte bits[1:0] = M1 : 0b01=CW(avant)  0b10=CCW(arrière)
//   dir_byte bits[3:2] = M2 : 0b01=CW(avant)  0b10=CCW(arrière)
[[maybe_unused]] constexpr uint8_t  I2C_REG_SPEED = 0x82;
[[maybe_unused]] constexpr uint8_t  I2C_REG_DIR   = 0xAA;

// Vitesse par défaut si aucun argument n'est fourni
constexpr double DEFAULT_RPM = 80.0;

[[maybe_unused]] constexpr double   PID_KP        = 1.9;
[[maybe_unused]] constexpr double   PID_KI        = 0.2;
[[maybe_unused]] constexpr double   PID_KD        = 0.02;

// Table de décodage quadrature (x4) : indice = (last_state << 2) | new_state
// Valeurs : +1 (avant), -1 (arrière), 0 (pas de mouvement / erreur).
constexpr int8_t QDEC_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers : stockage d'un double dans un atomic<uint64_t> (bit-cast sûr)
// ─────────────────────────────────────────────────────────────────────────────

static inline void atomic_store_f64(atomic<uint64_t>& a, double v) {
    static_assert(sizeof(double) == sizeof(uint64_t));
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    a.store(bits, memory_order_relaxed);
}

static inline double atomic_load_f64(const atomic<uint64_t>& a) {
    uint64_t bits = a.load(memory_order_relaxed);
    double v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

// ─────────────────────────────────────────────────────────────────────────────
// struct Encoder  — état partagé d'un encodeur
// ─────────────────────────────────────────────────────────────────────────────

struct Encoder {
    atomic<int32_t>  count          { 0 };
    atomic<uint8_t>  last_state     { 0 };
    atomic<uint64_t> speed_cps_bits { 0 };   // counts/s stocké en bits IEEE-754
    atomic<uint64_t> speed_rpm_bits { 0 };   // RPM     stocké en bits IEEE-754

    double speed_cps() const { return atomic_load_f64(speed_cps_bits); }
    double speed_rpm() const { return atomic_load_f64(speed_rpm_bits); }

    void set_speed(double cps, double rpm) {
        atomic_store_f64(speed_cps_bits, cps);
        atomic_store_f64(speed_rpm_bits, rpm);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// struct PidController  — régulateur PID discret avec anti-windup
// ─────────────────────────────────────────────────────────────────────────────

struct PidController {
    double kp, ki, kd;
    double integral   { 0.0 };
    double prev_error { 0.0 };
    double out_min, out_max;

    PidController(double kp, double ki, double kd, double out_min, double out_max)
        : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max) {}

    double update(double setpoint, double measured, double dt) {
        const double error      = setpoint - measured;
        integral               += error * dt;
        const double derivative = (dt > 0.0) ? (error - prev_error) / dt : 0.0;

        double output = kp * error + ki * integral + kd * derivative;
        output = clamp(output, out_min, out_max);

        // Anti-windup : borner l'intégrale pour qu'elle ne sature pas l'intégrateur
        if (ki != 0.0) {
            const double i_max = out_max / abs(ki);
            integral = clamp(integral, -i_max, i_max);
        }

        prev_error = error;
        return output;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Décodage quadrature
// ─────────────────────────────────────────────────────────────────────────────

static void update_encoder_from_state(Encoder& enc, uint8_t new_state) {
    const uint8_t last = enc.last_state.load(memory_order_relaxed);
    const uint8_t idx  = (last << 2) | new_state;
    enc.count.fetch_add(QDEC_TABLE[idx], memory_order_relaxed);
    enc.last_state.store(new_state, memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread vitesse : calcul à 10 Hz
// ─────────────────────────────────────────────────────────────────────────────

static void speed_thread_fn(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>&      enc1,
    const shared_ptr<Encoder>&      enc2)
{
    int32_t prev1  = enc1->count.load(memory_order_relaxed);
    int32_t prev2  = enc2->count.load(memory_order_relaxed);
    auto    t_prev = chrono::steady_clock::now();

    while (running->load(memory_order_relaxed)) {
        this_thread::sleep_for(chrono::milliseconds(100));

        const int32_t now1  = enc1->count.load(memory_order_relaxed);
        const int32_t now2  = enc2->count.load(memory_order_relaxed);
        const auto    t_now = chrono::steady_clock::now();

        const double dt = chrono::duration<double>(t_now - t_prev).count();
        if (dt > 0.0) {
            const double cps1 = (now1 - prev1) / dt;
            enc1->set_speed(cps1, cps1 / ENCODER_CPR * 60.0);

            const double cps2 = (now2 - prev2) / dt;
            enc2->set_speed(cps2, cps2 / ENCODER_CPR * 60.0);
        }

        prev1  = now1;
        prev2  = now2;
        t_prev = t_now;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Affichage statut en place (carriage return)
// ─────────────────────────────────────────────────────────────────────────────

static void print_status(const Encoder& enc1, const Encoder& enc2,
                          double tgt1, double tgt2) {
    cout << "\rENC1 [cible=" << showpos << fixed << setprecision(0) << tgt1
         << " RPM] pos=" << noshowpos << enc1.count.load(memory_order_relaxed)
         << " | " << showpos << setprecision(2) << enc1.speed_rpm() << " RPM"
         << " || ENC2 [cible=" << setprecision(0) << tgt2
         << " RPM] pos=" << noshowpos << enc2.count.load(memory_order_relaxed)
         << " | " << showpos << setprecision(2) << enc2.speed_rpm() << " RPM"
         << noshowpos << flush;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mode Raspberry Pi  (#ifdef RPI)
// ─────────────────────────────────────────────────────────────────────────────

#ifdef RPI

// ─────────────────────────────────────────────────────────────────────────────
// Encodage direction Grove L298P pour le registre 0xAA
// ─────────────────────────────────────────────────────────────────────────────

static uint8_t direction_byte(double rpm1, double rpm2) {
    const uint8_t d1 = (rpm1 >= 0.0) ? 0b01u : 0b10u;   // M1 : CW ou CCW
    const uint8_t d2 = (rpm2 >= 0.0) ? 0b01u : 0b10u;   // M2 : CW ou CCW
    return d1 | static_cast<uint8_t>(d2 << 2);
}

// Thread PID + écriture I2C
static void pid_i2c_thread_fn(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>&      enc1,
    const shared_ptr<Encoder>&      enc2,
    double                          target_rpm1,
    double                          target_rpm2)
{
    // ── Ouverture du bus I2C ─────────────────────────────────────────────────
    const int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        cerr << "Erreur I2C : ouverture /dev/i2c-1 impossible\n";
        return;
    }
    if (ioctl(fd, I2C_SLAVE, static_cast<long>(I2C_ADDR)) < 0) {
        cerr << "Erreur I2C : sélection adresse 0x" << hex << I2C_ADDR << dec << '\n';
        close(fd);
        return;
    }

    // ── Configuration initiale : direction + vitesse nulle ──────────────────
    {
        uint8_t dir_buf[] = { I2C_REG_DIR, direction_byte(target_rpm1, target_rpm2), 0x01 };
        if (::write(fd, dir_buf, sizeof(dir_buf)) != static_cast<ssize_t>(sizeof(dir_buf)))
            cerr << "Erreur I2C write reg 0xAA (direction initiale)\n";
    }

    PidController pid1(PID_KP, PID_KI, PID_KD, 0.0, 255.0);
    PidController pid2(PID_KP, PID_KI, PID_KD, 0.0, 255.0);
    auto    t_prev   = chrono::steady_clock::now();
    uint8_t last_dir = direction_byte(target_rpm1, target_rpm2);

    // Sature un double vers un octet 0..255
    auto saturate = [](double v) -> uint8_t {
        return static_cast<uint8_t>(clamp(round(v), 0.0, 255.0));
    };

    while (running->load(memory_order_relaxed)) {
        const auto   t_now = chrono::steady_clock::now();
        const double dt    = chrono::duration<double>(t_now - t_prev).count();
        t_prev = t_now;

        // PID sur la magnitude absolue (le signe = direction, géré séparément)
        const uint8_t cmd1 = saturate(pid1.update(abs(target_rpm1), abs(enc1->speed_rpm()), dt));
        const uint8_t cmd2 = saturate(pid2.update(abs(target_rpm2), abs(enc2->speed_rpm()), dt));

        // Direction : mise à jour du registre 0xAA uniquement si elle change
        const uint8_t cur_dir = direction_byte(target_rpm1, target_rpm2);
        if (cur_dir != last_dir) {
            uint8_t dir_buf[] = { I2C_REG_DIR, cur_dir, 0x01 };
            if (::write(fd, dir_buf, sizeof(dir_buf)) != static_cast<ssize_t>(sizeof(dir_buf)))
                cerr << "Erreur I2C write reg 0xAA (direction)\n";
            last_dir = cur_dir;
        }

        // Vitesses
        uint8_t buf[] = { I2C_REG_SPEED, cmd1, cmd2 };
        if (::write(fd, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf)))
            cerr << "Erreur I2C write reg 0x82\n";

        this_thread::sleep_for(chrono::milliseconds(PID_PERIOD_MS));
    }

    // Arrêt moteurs avant de quitter
    uint8_t stop_buf[] = { 0x82, 0x00, 0x00 };
    ::write(fd, stop_buf, sizeof(stop_buf));
    close(fd);
}

// Pointeurs globaux pour les ISR wiringPi (signature void(void) imposée)
static Encoder* g_enc1_ptr = nullptr;
static Encoder* g_enc2_ptr = nullptr;

static void gpio_alert_enc1() {
    if (g_enc1_ptr)
        update_encoder_from_state(*g_enc1_ptr,
            (static_cast<uint8_t>(digitalRead(ENC1_A)) << 1) |
             static_cast<uint8_t>(digitalRead(ENC1_B)));
}
static void gpio_alert_enc2() {
    if (g_enc2_ptr)
        update_encoder_from_state(*g_enc2_ptr,
            (static_cast<uint8_t>(digitalRead(ENC2_A)) << 1) |
             static_cast<uint8_t>(digitalRead(ENC2_B)));
}

static void run_rpi(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>&      enc1,
    const shared_ptr<Encoder>&      enc2,
    double                          target_rpm1,
    double                          target_rpm2)
{
    // wiringPiSetupGpio() utilise directement la numérotation BCM
    if (wiringPiSetupGpio() < 0) {
        cerr << "Erreur : wiringPiSetupGpio() a échoué\n";
        return;
    }

    for (int pin : { (int)ENC1_A, (int)ENC1_B, (int)ENC2_A, (int)ENC2_B }) {
        pinMode(pin, INPUT);
        pullUpDnControl(pin, PUD_UP);
    }

    // État initial des encodeurs
    enc1->last_state.store(
        (static_cast<uint8_t>(digitalRead(ENC1_A)) << 1) | static_cast<uint8_t>(digitalRead(ENC1_B)),
        memory_order_relaxed);
    enc2->last_state.store(
        (static_cast<uint8_t>(digitalRead(ENC2_A)) << 1) | static_cast<uint8_t>(digitalRead(ENC2_B)),
        memory_order_relaxed);

    g_enc1_ptr = enc1.get();
    g_enc2_ptr = enc2.get();

    // Enregistrement des ISR sur les fronts montants et descendants
    wiringPiISR(ENC1_A, INT_EDGE_BOTH, gpio_alert_enc1);
    wiringPiISR(ENC1_B, INT_EDGE_BOTH, gpio_alert_enc1);
    wiringPiISR(ENC2_A, INT_EDGE_BOTH, gpio_alert_enc2);
    wiringPiISR(ENC2_B, INT_EDGE_BOTH, gpio_alert_enc2);

    // Les jthread se joignent automatiquement à la fin de run_rpi()
    jthread speed_t([&] { speed_thread_fn(running, enc1, enc2); });
    jthread i2c_t  ([&] { pid_i2c_thread_fn(running, enc1, enc2, target_rpm1, target_rpm2); });

    cout << "Comptage + vitesse (10 Hz) avec PID I2C.\n"
         << "Consignes : M1=" << showpos << fixed << setprecision(0) << target_rpm1
         << " RPM, M2=" << target_rpm2 << noshowpos
         << " RPM. Ctrl+C pour arrêter.\n";

    while (running->load(memory_order_relaxed)) {
        print_status(*enc1, *enc2, target_rpm1, target_rpm2);
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    // wiringPi n'a pas de fonction de terminaison ; les ISR s'arrêtent avec le processus.
    // Destruction de i2c_t puis speed_t → join automatique
}

#endif // RPI

// ─────────────────────────────────────────────────────────────────────────────
// Mode virtuel — macOS / test sans GPIO  (#ifndef RPI)
// ─────────────────────────────────────────────────────────────────────────────

#ifndef RPI

static void run_virtual(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>&      enc1,
    const shared_ptr<Encoder>&      enc2,
    double                          target_rpm1,
    double                          target_rpm2)
{
    jthread speed_t([&] { speed_thread_fn(running, enc1, enc2); });

    cout << "Mode virtuel actif. Consignes : M1="
         << showpos << fixed << setprecision(0) << target_rpm1
         << " RPM, M2=" << target_rpm2 << noshowpos
         << " RPM. Ctrl+C pour arrêter.\n";

    // Séquences AB quadrature ; le sens suit le signe de la consigne
    constexpr uint8_t SEQ_FORWARD[4] = { 0b00, 0b01, 0b11, 0b10 };
    constexpr uint8_t SEQ_REVERSE[4] = { 0b00, 0b10, 0b11, 0b01 };
    const uint8_t* seq1 = (target_rpm1 >= 0.0) ? SEQ_FORWARD : SEQ_REVERSE;
    const uint8_t* seq2 = (target_rpm2 >= 0.0) ? SEQ_FORWARD : SEQ_REVERSE;
    size_t i1 = 0, i2 = 0;

    while (running->load(memory_order_relaxed)) {
        i1 = (i1 + 1) % 4;
        i2 = (i2 + 1) % 4;
        update_encoder_from_state(*enc1, seq1[i1]);
        update_encoder_from_state(*enc2, seq2[i2]);
        print_status(*enc1, *enc2, target_rpm1, target_rpm2);
        this_thread::sleep_for(chrono::milliseconds(20));
    }
    // Destruction de speed_t → join automatique
}

#endif // !RPI

// ─────────────────────────────────────────────────────────────────────────────
// Gestion signal SIGINT
// ─────────────────────────────────────────────────────────────────────────────

static shared_ptr<atomic<bool>> g_running;

static void sigint_handler(int) {
    if (g_running)
        g_running->store(false, memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    double target_rpm1 = DEFAULT_RPM;
    double target_rpm2 = DEFAULT_RPM;

    try {
        if (argc == 2) {
            target_rpm1 = target_rpm2 = stod(argv[1]);
        } else if (argc == 3) {
            target_rpm1 = stod(argv[1]);
            target_rpm2 = stod(argv[2]);
        } else if (argc > 3) {
            throw invalid_argument("trop d'arguments");
        }
    } catch (const exception& e) {
        cerr << "Erreur argument : " << e.what() << '\n'
             << "Usage : " << argv[0] << " [rpm_m1 [rpm_m2]]\n"
             << "  RPM positif = avant (CW), negatif = arriere (CCW)\n"
             << "  Exemple : " << argv[0] << " 80\n"
             << "            " << argv[0] << " 80 -60\n";
        return 1;
    }

    g_running = make_shared<atomic<bool>>(true);
    signal(SIGINT, sigint_handler);

    auto enc1 = make_shared<Encoder>();
    auto enc2 = make_shared<Encoder>();

#ifdef RPI
    run_rpi(g_running, enc1, enc2, target_rpm1, target_rpm2);
#else
    run_virtual(g_running, enc1, enc2, target_rpm1, target_rpm2);
#endif

    cout << "\nFinal ENC1=" << enc1->count.load(memory_order_relaxed)
         << " | ENC2=" << enc2->count.load(memory_order_relaxed) << '\n';

    return 0;
}
