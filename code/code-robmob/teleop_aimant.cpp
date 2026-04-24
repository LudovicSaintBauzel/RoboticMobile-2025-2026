// teleop_aimant.cpp — Teleoperation croisee avec comportement type aimant
//
// Idee:
// - Si les moteurs s'eloignent (distance qui augmente), ils s'attirent.
// - Si les moteurs se rapprochent (distance qui diminue), une resistance forte
//   s'oppose au mouvement (mode "mur magnetique").
//
// Compilation mode virtuel:
//   g++ -std=c++20 -O2 -Wall teleop_aimant.cpp -o teleop_aimant
//
// Compilation Raspberry Pi:
//   g++ -std=c++20 -O2 -Wall -DRPI teleop_aimant.cpp -o teleop_aimant -lwiringPi -lpthread
//
// Usage:
//   ./teleop_aimant             -> vitesse max 120 RPM
//   ./teleop_aimant 80          -> limite de vitesse a 80 RPM

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#ifdef RPI
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#  include <linux/i2c-dev.h>
#  include <wiringPi.h>
#endif

using namespace std;

[[maybe_unused]] constexpr uint8_t ENC1_A = 19;
[[maybe_unused]] constexpr uint8_t ENC1_B = 18;
[[maybe_unused]] constexpr uint8_t ENC2_A = 17;
[[maybe_unused]] constexpr uint8_t ENC2_B = 16;

constexpr double ENCODER_CPR = 1440.0;

[[maybe_unused]] constexpr int I2C_ADDR = 0x0F;
[[maybe_unused]] constexpr uint32_t CTRL_PERIOD_MS = 10;
[[maybe_unused]] constexpr uint8_t I2C_REG_SPEED = 0x82;
[[maybe_unused]] constexpr uint8_t I2C_REG_DIR = 0xAA;

constexpr double DEFAULT_MAX_RPM = 120.0;
constexpr double POSITION_KP_RPM_PER_COUNT = 0.30;
constexpr int32_t POSITION_DEADBAND_COUNTS = 4;
constexpr int32_t VIRTUAL_INITIAL_OFFSET_COUNTS = 300;
constexpr int32_t RESIST_OFFSET_COUNTS = 320;
constexpr double DISTANCE_RATE_RESIST_ENTER = -12.0; // tops/s
constexpr double DISTANCE_RATE_ATTRACT_ENTER = 12.0; // tops/s
constexpr double VEL_OPPOSE_EPS = 0.5;    // tops/s

[[maybe_unused]] constexpr double SPEED_PID_KP = 5.9;
[[maybe_unused]] constexpr double SPEED_PID_KI = 0.0;
[[maybe_unused]] constexpr double SPEED_PID_KD = 0.04;

constexpr int8_t QDEC_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline void atomic_store_f64(atomic<uint64_t>& a, double v) {
    static_assert(sizeof(double) == sizeof(uint64_t));
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    a.store(bits, memory_order_relaxed);
}

static inline double atomic_load_f64(const atomic<uint64_t>& a) {
    const uint64_t bits = a.load(memory_order_relaxed);
    double value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

struct Encoder {
    atomic<int32_t> count{0};
    atomic<uint8_t> last_state{0};
    atomic<uint64_t> speed_rpm_bits{0};

    double speed_rpm() const { return atomic_load_f64(speed_rpm_bits); }

    void set_speed_rpm(double rpm) {
        atomic_store_f64(speed_rpm_bits, rpm);
    }
};

struct PidController {
    double kp;
    double ki;
    double kd;
    double integral{0.0};
    double prev_error{0.0};
    double out_min;
    double out_max;

    PidController(double kp, double ki, double kd, double out_min, double out_max)
        : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max) {}

    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }

    double update(double setpoint, double measured, double dt) {
        const double error = setpoint - measured;
        integral += error * dt;
        const double derivative = (dt > 0.0) ? (error - prev_error) / dt : 0.0;

        double output = kp * error + ki * integral + kd * derivative;
        output = clamp(output, out_min, out_max);

        if (ki != 0.0) {
            const double i_max = out_max / abs(ki);
            integral = clamp(integral, -i_max, i_max);
        }

        prev_error = error;
        return output;
    }
};

enum class MagnetMode : uint8_t {
    Attract = 0,
    Resist = 1
};

struct MagnetState {
    atomic<int32_t> target_pos1{0};
    atomic<int32_t> target_pos2{0};
    atomic<int32_t> error1{0};
    atomic<int32_t> error2{0};
    atomic<uint8_t> cmd1{0};
    atomic<uint8_t> cmd2{0};
    atomic<uint64_t> target_rpm1_bits{0};
    atomic<uint64_t> target_rpm2_bits{0};
    atomic<uint8_t> mode{static_cast<uint8_t>(MagnetMode::Attract)};
    atomic<uint64_t> distance_bits{0};
    atomic<uint64_t> distance_rate_bits{0};

    void set_target_rpms(double rpm1, double rpm2) {
        atomic_store_f64(target_rpm1_bits, rpm1);
        atomic_store_f64(target_rpm2_bits, rpm2);
    }

    void set_distance(double distance, double distance_rate) {
        atomic_store_f64(distance_bits, distance);
        atomic_store_f64(distance_rate_bits, distance_rate);
    }

    double target_rpm1() const { return atomic_load_f64(target_rpm1_bits); }
    double target_rpm2() const { return atomic_load_f64(target_rpm2_bits); }
    double distance() const { return atomic_load_f64(distance_bits); }
    double distance_rate() const { return atomic_load_f64(distance_rate_bits); }
};

static void update_encoder_from_state(Encoder& enc, uint8_t new_state) {
    const uint8_t last = enc.last_state.load(memory_order_relaxed);
    const uint8_t idx = static_cast<uint8_t>((last << 2) | new_state);
    enc.count.fetch_add(QDEC_TABLE[idx], memory_order_relaxed);
    enc.last_state.store(new_state, memory_order_relaxed);
}

static void speed_thread_fn(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>& enc1,
    const shared_ptr<Encoder>& enc2)
{
    int32_t prev1 = enc1->count.load(memory_order_relaxed);
    int32_t prev2 = enc2->count.load(memory_order_relaxed);
    auto t_prev = chrono::steady_clock::now();

    while (running->load(memory_order_relaxed)) {
        this_thread::sleep_for(chrono::milliseconds(10));

        const int32_t now1 = enc1->count.load(memory_order_relaxed);
        const int32_t now2 = enc2->count.load(memory_order_relaxed);
        const auto t_now = chrono::steady_clock::now();
        const double dt = chrono::duration<double>(t_now - t_prev).count();

        if (dt > 0.0) {
            const double cps1 = (now1 - prev1) / dt;
            const double cps2 = (now2 - prev2) / dt;
            enc1->set_speed_rpm(cps1 / ENCODER_CPR * 60.0);
            enc2->set_speed_rpm(cps2 / ENCODER_CPR * 60.0);
        }

        prev1 = now1;
        prev2 = now2;
        t_prev = t_now;
    }
}

static double position_error_to_rpm(int32_t error_counts, double max_rpm) {
    if (abs(error_counts) <= POSITION_DEADBAND_COUNTS) {
        return 0.0;
    }

    const double rpm = -static_cast<double>(error_counts) * POSITION_KP_RPM_PER_COUNT;
    return clamp(rpm, -max_rpm, max_rpm);
}

[[maybe_unused]] static uint8_t direction_byte(double rpm1, double rpm2) {
    const uint8_t d1 = (rpm1 >= 0.0) ? 0b01u : 0b10u;
    const uint8_t d2 = (rpm2 >= 0.0) ? 0b01u : 0b10u;
    return static_cast<uint8_t>(d1 | (d2 << 2));
}

static int sign_from_velocity(double v) {
    if (v > VEL_OPPOSE_EPS) {
        return 1;
    }
    if (v < -VEL_OPPOSE_EPS) {
        return -1;
    }
    return 0;
}

static void compute_magnet_targets(
    int32_t pos1,
    int32_t pos2,
    double vel1_counts_s,
    double vel2_counts_s,
    double distance_rate,
    int32_t& target_pos1,
    int32_t& target_pos2,
    MagnetMode& mode,
    int& resist_dir1,
    int& resist_dir2)
{
    // Hysteresis: on ne change de mode que si la vitesse de variation de la
    // distance franchit des seuils distincts entree/sortie.
    if (mode == MagnetMode::Attract) {
        if (distance_rate < DISTANCE_RATE_RESIST_ENTER) {
            mode = MagnetMode::Resist;
            resist_dir1 = sign_from_velocity(vel1_counts_s);
            resist_dir2 = sign_from_velocity(vel2_counts_s);
        }
    } else {
        if (resist_dir1 == 0 && abs(vel1_counts_s) > VEL_OPPOSE_EPS) {
            resist_dir1 = sign_from_velocity(vel1_counts_s);
        }
        if (resist_dir2 == 0 && abs(vel2_counts_s) > VEL_OPPOSE_EPS) {
            resist_dir2 = sign_from_velocity(vel2_counts_s);
        }

        if (resist_dir1 > 0 && vel1_counts_s <= VEL_OPPOSE_EPS) {
            resist_dir1 = 0;
        } else if (resist_dir1 < 0 && vel1_counts_s >= -VEL_OPPOSE_EPS) {
            resist_dir1 = 0;
        }
        if (resist_dir2 > 0 && vel2_counts_s <= VEL_OPPOSE_EPS) {
            resist_dir2 = 0;
        } else if (resist_dir2 < 0 && vel2_counts_s >= -VEL_OPPOSE_EPS) {
            resist_dir2 = 0;
        }

        if (resist_dir1 == 0 && resist_dir2 == 0 && distance_rate > DISTANCE_RATE_ATTRACT_ENTER) {
            mode = MagnetMode::Attract;
        }
    }

    if (mode == MagnetMode::Resist) {
        target_pos1 = pos1 - resist_dir1 * RESIST_OFFSET_COUNTS;
        target_pos2 = pos2 - resist_dir2 * RESIST_OFFSET_COUNTS;
    } else {
        resist_dir1 = 0;
        resist_dir2 = 0;
        target_pos1 = pos2;
        target_pos2 = pos1;
    }
}

static void print_status(const Encoder& enc1, const Encoder& enc2, const MagnetState& state) {
    const auto current_mode = static_cast<MagnetMode>(state.mode.load(memory_order_relaxed));
    const char* mode_text = (current_mode == MagnetMode::Resist) ? "RESIST" : "ATTRACT";

    cout << "\r[" << mode_text << "]"
         << " d=" << fixed << setprecision(1) << state.distance()
         << " dd=" << showpos << fixed << setprecision(1) << state.distance_rate() << noshowpos
         << " | M1 pos=" << enc1.count.load(memory_order_relaxed)
         << " cible=" << state.target_pos1.load(memory_order_relaxed)
         << " err=" << showpos << state.error1.load(memory_order_relaxed)
         << " rpm=" << fixed << setprecision(2) << enc1.speed_rpm()
         << " cmd=" << noshowpos << static_cast<int>(state.cmd1.load(memory_order_relaxed))
         << " || M2 pos=" << enc2.count.load(memory_order_relaxed)
         << " cible=" << state.target_pos2.load(memory_order_relaxed)
         << " err=" << showpos << state.error2.load(memory_order_relaxed)
         << " rpm=" << fixed << setprecision(2) << enc2.speed_rpm()
         << " cmd=" << noshowpos << static_cast<int>(state.cmd2.load(memory_order_relaxed))
         << " | consignes RPM=(" << showpos << fixed << setprecision(1)
         << state.target_rpm1() << ", " << state.target_rpm2() << ")"
         << noshowpos << flush;
}

#ifdef RPI

static void teleop_aimant_i2c_thread_fn(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>& enc1,
    const shared_ptr<Encoder>& enc2,
    const shared_ptr<MagnetState>& state,
    double max_rpm)
{
    const int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        cerr << "Erreur I2C : ouverture /dev/i2c-1 impossible\n";
        running->store(false, memory_order_relaxed);
        return;
    }
    if (ioctl(fd, I2C_SLAVE, static_cast<long>(I2C_ADDR)) < 0) {
        cerr << "Erreur I2C : sélection adresse 0x" << hex << I2C_ADDR << dec << '\n';
        close(fd);
        running->store(false, memory_order_relaxed);
        return;
    }

    uint8_t dir_buf[] = {I2C_REG_DIR, direction_byte(0.0, 0.0), 0x01};
    if (::write(fd, dir_buf, sizeof(dir_buf)) != static_cast<ssize_t>(sizeof(dir_buf))) {
        cerr << "Erreur I2C write reg 0xAA (direction initiale)\n";
    }

    PidController speed_pid1(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 0.0, 255.0);
    PidController speed_pid2(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 0.0, 255.0);
    uint8_t last_dir = direction_byte(0.0, 0.0);

    auto saturate_u8 = [](double value) {
        return static_cast<uint8_t>(clamp(round(value), 0.0, 255.0));
    };

    int32_t prev_pos1 = enc1->count.load(memory_order_relaxed);
    int32_t prev_pos2 = enc2->count.load(memory_order_relaxed);
    double prev_distance = static_cast<double>(abs(prev_pos2 - prev_pos1));
    MagnetMode mode = MagnetMode::Attract;
    int resist_dir1 = 0;
    int resist_dir2 = 0;
    auto t_prev = chrono::steady_clock::now();

    while (running->load(memory_order_relaxed)) {
        const auto t_now = chrono::steady_clock::now();
        const double dt = chrono::duration<double>(t_now - t_prev).count();
        t_prev = t_now;
        if (dt <= 0.0) {
            continue;
        }

        const int32_t pos1 = enc1->count.load(memory_order_relaxed);
        const int32_t pos2 = enc2->count.load(memory_order_relaxed);

        const double vel1_counts_s = (pos1 - prev_pos1) / dt;
        const double vel2_counts_s = (pos2 - prev_pos2) / dt;

        const double distance = static_cast<double>(abs(pos2 - pos1));
        const double distance_rate = (distance - prev_distance) / dt;

        int32_t target_pos1 = pos2;
        int32_t target_pos2 = pos1;

        compute_magnet_targets(
            pos1, pos2, vel1_counts_s, vel2_counts_s, distance_rate,
            target_pos1, target_pos2, mode, resist_dir1, resist_dir2);

        const int32_t error1 = target_pos1 - pos1;
        const int32_t error2 = target_pos2 - pos2;

        const double target_rpm1 = position_error_to_rpm(error1, max_rpm);
        const double target_rpm2 = position_error_to_rpm(error2, max_rpm);

        uint8_t cmd1 = 0;
        uint8_t cmd2 = 0;

        if (target_rpm1 == 0.0) {
            speed_pid1.reset();
        } else {
            cmd1 = saturate_u8(speed_pid1.update(abs(target_rpm1), abs(enc1->speed_rpm()), dt));
        }

        if (target_rpm2 == 0.0) {
            speed_pid2.reset();
        } else {
            cmd2 = saturate_u8(speed_pid2.update(abs(target_rpm2), abs(enc2->speed_rpm()), dt));
        }

        const uint8_t cur_dir = direction_byte(target_rpm1, target_rpm2);
        if (cur_dir != last_dir) {
            uint8_t update_dir[] = {I2C_REG_DIR, cur_dir, 0x01};
            if (::write(fd, update_dir, sizeof(update_dir)) != static_cast<ssize_t>(sizeof(update_dir))) {
                cerr << "Erreur I2C write reg 0xAA (direction)\n";
            }
            last_dir = cur_dir;
        }

        uint8_t speed_buf[] = {I2C_REG_SPEED, cmd1, cmd2};
        if (::write(fd, speed_buf, sizeof(speed_buf)) != static_cast<ssize_t>(sizeof(speed_buf))) {
            cerr << "Erreur I2C write reg 0x82\n";
        }

        state->target_pos1.store(target_pos1, memory_order_relaxed);
        state->target_pos2.store(target_pos2, memory_order_relaxed);
        state->error1.store(error1, memory_order_relaxed);
        state->error2.store(error2, memory_order_relaxed);
        state->cmd1.store(cmd1, memory_order_relaxed);
        state->cmd2.store(cmd2, memory_order_relaxed);
        state->set_target_rpms(target_rpm1, target_rpm2);
        state->mode.store(static_cast<uint8_t>(mode), memory_order_relaxed);
        state->set_distance(distance, distance_rate);

        prev_pos1 = pos1;
        prev_pos2 = pos2;
        prev_distance = distance;

        this_thread::sleep_for(chrono::milliseconds(CTRL_PERIOD_MS));
    }

    uint8_t stop_buf[] = {I2C_REG_SPEED, 0x00, 0x00};
    ::write(fd, stop_buf, sizeof(stop_buf));
    close(fd);
}

static Encoder* g_enc1_ptr = nullptr;
static Encoder* g_enc2_ptr = nullptr;

static void gpio_alert_enc1() {
    if (g_enc1_ptr) {
        update_encoder_from_state(
            *g_enc1_ptr,
            static_cast<uint8_t>((static_cast<uint8_t>(digitalRead(ENC1_A)) << 1) |
                                 static_cast<uint8_t>(digitalRead(ENC1_B))));
    }
}

static void gpio_alert_enc2() {
    if (g_enc2_ptr) {
        update_encoder_from_state(
            *g_enc2_ptr,
            static_cast<uint8_t>((static_cast<uint8_t>(digitalRead(ENC2_A)) << 1) |
                                 static_cast<uint8_t>(digitalRead(ENC2_B))));
    }
}

static void run_rpi(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>& enc1,
    const shared_ptr<Encoder>& enc2,
    double max_rpm)
{
    if (wiringPiSetupGpio() < 0) {
        cerr << "Erreur : wiringPiSetupGpio() a échoué\n";
        return;
    }

    for (int pin : {(int)ENC1_A, (int)ENC1_B, (int)ENC2_A, (int)ENC2_B}) {
        pinMode(pin, INPUT);
        pullUpDnControl(pin, PUD_UP);
    }

    enc1->last_state.store(
        static_cast<uint8_t>((static_cast<uint8_t>(digitalRead(ENC1_A)) << 1) |
                             static_cast<uint8_t>(digitalRead(ENC1_B))),
        memory_order_relaxed);
    enc2->last_state.store(
        static_cast<uint8_t>((static_cast<uint8_t>(digitalRead(ENC2_A)) << 1) |
                             static_cast<uint8_t>(digitalRead(ENC2_B))),
        memory_order_relaxed);

    g_enc1_ptr = enc1.get();
    g_enc2_ptr = enc2.get();

    wiringPiISR(ENC1_A, INT_EDGE_BOTH, gpio_alert_enc1);
    wiringPiISR(ENC1_B, INT_EDGE_BOTH, gpio_alert_enc1);
    wiringPiISR(ENC2_A, INT_EDGE_BOTH, gpio_alert_enc2);
    wiringPiISR(ENC2_B, INT_EDGE_BOTH, gpio_alert_enc2);

    auto state = make_shared<MagnetState>();

    jthread speed_t([&] { speed_thread_fn(running, enc1, enc2); });
    jthread ctrl_t([&] { teleop_aimant_i2c_thread_fn(running, enc1, enc2, state, max_rpm); });

    cout << "Mode aimant actif.\n"
         << "- Les moteurs qui s'eloignent sont attires.\n"
         << "- Les moteurs qui se rapprochent rencontrent une forte opposition.\n"
         << "Limitation de vitesse : +/-" << fixed << setprecision(0) << max_rpm
         << " RPM. Ctrl+C pour arrêter.\n";

    while (running->load(memory_order_relaxed)) {
        print_status(*enc1, *enc2, *state);
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

#else

static void run_virtual(
    const shared_ptr<atomic<bool>>& running,
    const shared_ptr<Encoder>& enc1,
    const shared_ptr<Encoder>& enc2,
    double max_rpm)
{
    auto state = make_shared<MagnetState>();
    jthread speed_t([&] { speed_thread_fn(running, enc1, enc2); });

    enc1->count.store(VIRTUAL_INITIAL_OFFSET_COUNTS, memory_order_relaxed);

    cout << "Mode virtuel aimant actif. Decalage initial de " << VIRTUAL_INITIAL_OFFSET_COUNTS
         << " tops sur M1.\n"
         << "Limitation de vitesse : +/-" << fixed << setprecision(0) << max_rpm
         << " RPM. Ctrl+C pour arrêter.\n";

    constexpr uint8_t SEQ_FORWARD[4] = {0b00, 0b01, 0b11, 0b10};
    constexpr uint8_t SEQ_REVERSE[4] = {0b00, 0b10, 0b11, 0b01};
    size_t phase1 = 0;
    size_t phase2 = 0;
    double residual_steps1 = 0.0;
    double residual_steps2 = 0.0;

    int32_t prev_pos1 = enc1->count.load(memory_order_relaxed);
    int32_t prev_pos2 = enc2->count.load(memory_order_relaxed);
    double prev_distance = static_cast<double>(abs(prev_pos2 - prev_pos1));
    MagnetMode mode = MagnetMode::Attract;
    int resist_dir1 = 0;
    int resist_dir2 = 0;
    auto t_prev = chrono::steady_clock::now();

    while (running->load(memory_order_relaxed)) {
        const auto t_now = chrono::steady_clock::now();
        const double dt = chrono::duration<double>(t_now - t_prev).count();
        t_prev = t_now;
        if (dt <= 0.0) {
            continue;
        }

        const int32_t pos1 = enc1->count.load(memory_order_relaxed);
        const int32_t pos2 = enc2->count.load(memory_order_relaxed);

        const double vel1_counts_s = (pos1 - prev_pos1) / dt;
        const double vel2_counts_s = (pos2 - prev_pos2) / dt;

        const double distance = static_cast<double>(abs(pos2 - pos1));
        const double distance_rate = (distance - prev_distance) / dt;

        int32_t target_pos1 = pos2;
        int32_t target_pos2 = pos1;
        compute_magnet_targets(
            pos1, pos2, vel1_counts_s, vel2_counts_s, distance_rate,
            target_pos1, target_pos2, mode, resist_dir1, resist_dir2);

        const int32_t error1 = target_pos1 - pos1;
        const int32_t error2 = target_pos2 - pos2;
        const double target_rpm1 = position_error_to_rpm(error1, max_rpm);
        const double target_rpm2 = position_error_to_rpm(error2, max_rpm);

        const double steps1 = abs(target_rpm1) * ENCODER_CPR / 60.0 * dt;
        const double steps2 = abs(target_rpm2) * ENCODER_CPR / 60.0 * dt;
        residual_steps1 += steps1;
        residual_steps2 += steps2;

        const uint8_t* seq1 = (target_rpm1 >= 0.0) ? SEQ_FORWARD : SEQ_REVERSE;
        const uint8_t* seq2 = (target_rpm2 >= 0.0) ? SEQ_FORWARD : SEQ_REVERSE;

        while (residual_steps1 >= 1.0) {
            phase1 = (phase1 + 1) % 4;
            update_encoder_from_state(*enc1, seq1[phase1]);
            residual_steps1 -= 1.0;
        }
        while (residual_steps2 >= 1.0) {
            phase2 = (phase2 + 1) % 4;
            update_encoder_from_state(*enc2, seq2[phase2]);
            residual_steps2 -= 1.0;
        }

        state->target_pos1.store(target_pos1, memory_order_relaxed);
        state->target_pos2.store(target_pos2, memory_order_relaxed);
        state->error1.store(error1, memory_order_relaxed);
        state->error2.store(error2, memory_order_relaxed);
        state->cmd1.store(static_cast<uint8_t>(round(clamp(abs(target_rpm1) / max_rpm * 255.0, 0.0, 255.0))), memory_order_relaxed);
        state->cmd2.store(static_cast<uint8_t>(round(clamp(abs(target_rpm2) / max_rpm * 255.0, 0.0, 255.0))), memory_order_relaxed);
        state->set_target_rpms(target_rpm1, target_rpm2);
        state->mode.store(static_cast<uint8_t>(mode), memory_order_relaxed);
        state->set_distance(distance, distance_rate);

        prev_pos1 = pos1;
        prev_pos2 = pos2;
        prev_distance = distance;

        print_status(*enc1, *enc2, *state);
        this_thread::sleep_for(chrono::milliseconds(CTRL_PERIOD_MS));
    }
}

#endif

static shared_ptr<atomic<bool>> g_running;

static void sigint_handler(int) {
    if (g_running) {
        g_running->store(false, memory_order_relaxed);
    }
}

int main(int argc, char* argv[]) {
    double max_rpm = DEFAULT_MAX_RPM;

    try {
        if (argc == 2) {
            max_rpm = abs(stod(argv[1]));
        } else if (argc > 2) {
            throw invalid_argument("trop d'arguments");
        }
        if (max_rpm <= 0.0) {
            throw invalid_argument("la vitesse max doit etre strictement positive");
        }
    } catch (const exception& e) {
        cerr << "Erreur argument : " << e.what() << '\n'
             << "Usage : " << argv[0] << " [vitesse_max_rpm]\n"
             << "  Exemple : " << argv[0] << " 80\n";
        return 1;
    }

    g_running = make_shared<atomic<bool>>(true);
    signal(SIGINT, sigint_handler);

    auto enc1 = make_shared<Encoder>();
    auto enc2 = make_shared<Encoder>();

#ifdef RPI
    run_rpi(g_running, enc1, enc2, max_rpm);
#else
    run_virtual(g_running, enc1, enc2, max_rpm);
#endif

    cout << "\nFinal ENC1=" << enc1->count.load(memory_order_relaxed)
         << " | ENC2=" << enc2->count.load(memory_order_relaxed) << '\n';
    return 0;
}
