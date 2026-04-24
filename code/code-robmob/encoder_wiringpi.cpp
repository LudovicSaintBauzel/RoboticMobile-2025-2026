// encoder_wiringpi.cpp
#include <cstdint>
#include <atomic>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <stop_token>
#include <wiringPi.h>

using namespace std;

#define ENC1_A 18   // GPIO BCM 18 (pin 12)
#define ENC1_B 19   // GPIO BCM 19 (pin 35)
#define ENC2_A 16   // GPIO BCM 16 (pin 36)
#define ENC2_B 17   // GPIO BCM 17 (pin 11)

// Nombre de "counts" par tour en mode x4 (à adapter à ton codeur)
#define ENCODER_CPR 1024.0

volatile sig_atomic_t stop_from_signal = 0;

struct encoder_t {
    atomic<int32_t> count{0};
    atomic<uint8_t> last_state{0};
    atomic<double> speed_cps{0.0};
    atomic<double> speed_rpm{0.0};
};

encoder_t enc1;
encoder_t enc2;

static const int8_t qdec_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

void handle_sigint(int sig) {
    (void)sig;
    stop_from_signal = 1;
}

static void update_encoder(encoder_t& enc, int pin_a, int pin_b) {
    uint8_t a = (uint8_t)digitalRead(pin_a);
    uint8_t b = (uint8_t)digitalRead(pin_b);
    uint8_t new_state = (uint8_t)((a << 1) | b);

    uint8_t last_state = enc.last_state.load(memory_order_relaxed);
    uint8_t idx = (uint8_t)((last_state << 2) | new_state);
    enc.count.fetch_add(qdec_table[idx], memory_order_relaxed);
    enc.last_state.store(new_state, memory_order_relaxed);
}

void encoder1_isr_a(void) {
    update_encoder(enc1, ENC1_A, ENC1_B);
}

void encoder1_isr_b(void) {
    update_encoder(enc1, ENC1_A, ENC1_B);
}

void encoder2_isr_a(void) {
    update_encoder(enc2, ENC2_A, ENC2_B);
}

void encoder2_isr_b(void) {
    update_encoder(enc2, ENC2_A, ENC2_B);
}

void speed_thread_fn(stop_token stoken) {
    using clock = chrono::steady_clock;
    using seconds_f = chrono::duration<double>;

    int32_t prev_count_1 = enc1.count.load(memory_order_relaxed);
    int32_t prev_count_2 = enc2.count.load(memory_order_relaxed);
    auto t_prev = clock::now();

    while (!stoken.stop_requested()) {
        this_thread::sleep_for(chrono::milliseconds(100)); // 10 Hz

        int32_t now_count_1 = enc1.count.load(memory_order_relaxed);
        int32_t now_count_2 = enc2.count.load(memory_order_relaxed);
        auto t_now = clock::now();

        double dt = chrono::duration_cast<seconds_f>(t_now - t_prev).count();
        int32_t dc_1 = now_count_1 - prev_count_1;
        int32_t dc_2 = now_count_2 - prev_count_2;

        if (dt > 0.0) {
            double speed1 = (double)dc_1 / dt;
            enc1.speed_cps.store(speed1, memory_order_relaxed);
            enc1.speed_rpm.store((speed1 / ENCODER_CPR) * 60.0, memory_order_relaxed);

            double speed2 = (double)dc_2 / dt;
            enc2.speed_cps.store(speed2, memory_order_relaxed);
            enc2.speed_rpm.store((speed2 / ENCODER_CPR) * 60.0, memory_order_relaxed);
        }

        prev_count_1 = now_count_1;
        prev_count_2 = now_count_2;
        t_prev = t_now;
    }
}

int main(void) {
    signal(SIGINT, handle_sigint);

    if (wiringPiSetupGpio() == -1) {
        cerr << "Erreur: wiringPiSetupGpio a echoue." << endl;
        return 1;
    }

    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);
    pullUpDnControl(ENC1_A, PUD_UP);
    pullUpDnControl(ENC1_B, PUD_UP);
    pullUpDnControl(ENC2_A, PUD_UP);
    pullUpDnControl(ENC2_B, PUD_UP);

    enc1.last_state.store((uint8_t)((digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B)), memory_order_relaxed);
    enc2.last_state.store((uint8_t)((digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B)), memory_order_relaxed);

    if (wiringPiISR(ENC1_A, INT_EDGE_BOTH, &encoder1_isr_a) < 0) {
        cerr << "Erreur: wiringPiISR ENC1_A a echoue." << endl;
        return 1;
    }
    if (wiringPiISR(ENC1_B, INT_EDGE_BOTH, &encoder1_isr_b) < 0) {
        cerr << "Erreur: wiringPiISR ENC1_B a echoue." << endl;
        return 1;
    }
    if (wiringPiISR(ENC2_A, INT_EDGE_BOTH, &encoder2_isr_a) < 0) {
        cerr << "Erreur: wiringPiISR ENC2_A a echoue." << endl;
        return 1;
    }
    if (wiringPiISR(ENC2_B, INT_EDGE_BOTH, &encoder2_isr_b) < 0) {
        cerr << "Erreur: wiringPiISR ENC2_B a echoue." << endl;
        return 1;
    }

    jthread speed_thread(speed_thread_fn);

    cout << "Comptage + vitesse (10 Hz) pour 2 codeurs. Ctrl+C pour arreter." << endl;
    cout << fixed << setprecision(2);
    while (!stop_from_signal) {
        int32_t c1 = enc1.count.load(memory_order_relaxed);
        int32_t c2 = enc2.count.load(memory_order_relaxed);
        double cps1 = enc1.speed_cps.load(memory_order_relaxed);
        double rpm1 = enc1.speed_rpm.load(memory_order_relaxed);
        double cps2 = enc2.speed_cps.load(memory_order_relaxed);
        double rpm2 = enc2.speed_rpm.load(memory_order_relaxed);

        cout << "\rENC1 pos=" << static_cast<long>(c1)
             << " | v=" << cps1 << " counts/s | " << rpm1 << " RPM || "
             << "ENC2 pos=" << static_cast<long>(c2)
             << " | v=" << cps2 << " counts/s | " << rpm2 << " RPM"
             << flush;
        this_thread::sleep_for(chrono::milliseconds(100)); // affichage 10 Hz
    }

    speed_thread.request_stop();

    cout << "\nFinal ENC1=" << static_cast<long>(enc1.count.load(memory_order_relaxed))
         << " | ENC2=" << static_cast<long>(enc2.count.load(memory_order_relaxed)) << endl;
    return 0;
}