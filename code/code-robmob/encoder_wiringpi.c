// encoder_wiringpi.c
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <wiringPi.h>

#define ENC1_A 18   // GPIO BCM 18 (pin 12)
#define ENC1_B 19   // GPIO BCM 19 (pin 35)
#define ENC2_A 16   // GPIO BCM 16 (pin 36)
#define ENC2_B 17   // GPIO BCM 17 (pin 11)

// Nombre de "counts" par tour en mode x4 (à adapter à ton codeur)
#define ENCODER_CPR 1024.0

volatile sig_atomic_t keep_running = 1;

typedef struct {
    volatile int32_t count;
    volatile uint8_t last_state;
    volatile double speed_cps;
    volatile double speed_rpm;
} encoder_t;

volatile encoder_t enc1 = {0};
volatile encoder_t enc2 = {0};

static const int8_t qdec_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

static void update_encoder(volatile encoder_t *enc, int pin_a, int pin_b) {
    uint8_t a = (uint8_t)digitalRead(pin_a);
    uint8_t b = (uint8_t)digitalRead(pin_b);
    uint8_t new_state = (uint8_t)((a << 1) | b);

    uint8_t idx = (uint8_t)((enc->last_state << 2) | new_state);
    enc->count += qdec_table[idx];
    enc->last_state = new_state;
}

void encoder1_isr_a(void) {
    update_encoder(&enc1, ENC1_A, ENC1_B);
}

void encoder1_isr_b(void) {
    update_encoder(&enc1, ENC1_A, ENC1_B);
}

void encoder2_isr_a(void) {
    update_encoder(&enc2, ENC2_A, ENC2_B);
}

void encoder2_isr_b(void) {
    update_encoder(&enc2, ENC2_A, ENC2_B);
}

static double elapsed_s(const struct timespec *t0, const struct timespec *t1) {
    return (double)(t1->tv_sec - t0->tv_sec)
         + (double)(t1->tv_nsec - t0->tv_nsec) / 1e9;
}

void* speed_thread_fn(void *arg) {
    (void)arg;

    int32_t prev_count_1 = enc1.count;
    int32_t prev_count_2 = enc2.count;
    struct timespec t_prev, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_prev);

    while (keep_running) {
        usleep(100000); // 10 Hz

        int32_t now_count_1 = enc1.count;
        int32_t now_count_2 = enc2.count;
        clock_gettime(CLOCK_MONOTONIC, &t_now);

        double dt = elapsed_s(&t_prev, &t_now);
        int32_t dc_1 = now_count_1 - prev_count_1;
        int32_t dc_2 = now_count_2 - prev_count_2;

        if (dt > 0.0) {
            enc1.speed_cps = (double)dc_1 / dt;
            enc1.speed_rpm = (enc1.speed_cps / ENCODER_CPR) * 60.0;

            enc2.speed_cps = (double)dc_2 / dt;
            enc2.speed_rpm = (enc2.speed_cps / ENCODER_CPR) * 60.0;
        }

        prev_count_1 = now_count_1;
        prev_count_2 = now_count_2;
        t_prev = t_now;
    }

    return NULL;
}

int main(void) {
    signal(SIGINT, handle_sigint);

    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Erreur: wiringPiSetupGpio a echoue.\n");
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

    enc1.last_state = (uint8_t)((digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B));
    enc2.last_state = (uint8_t)((digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B));

    if (wiringPiISR(ENC1_A, INT_EDGE_BOTH, &encoder1_isr_a) < 0) {
        fprintf(stderr, "Erreur: wiringPiISR ENC1_A a echoue.\n");
        return 1;
    }
    if (wiringPiISR(ENC1_B, INT_EDGE_BOTH, &encoder1_isr_b) < 0) {
        fprintf(stderr, "Erreur: wiringPiISR ENC1_B a echoue.\n");
        return 1;
    }
    if (wiringPiISR(ENC2_A, INT_EDGE_BOTH, &encoder2_isr_a) < 0) {
        fprintf(stderr, "Erreur: wiringPiISR ENC2_A a echoue.\n");
        return 1;
    }
    if (wiringPiISR(ENC2_B, INT_EDGE_BOTH, &encoder2_isr_b) < 0) {
        fprintf(stderr, "Erreur: wiringPiISR ENC2_B a echoue.\n");
        return 1;
    }

    pthread_t speed_thread;
    if (pthread_create(&speed_thread, NULL, speed_thread_fn, NULL) != 0) {
        fprintf(stderr, "Erreur: creation du thread vitesse impossible.\n");
        return 1;
    }

    printf("Comptage + vitesse (10 Hz) pour 2 codeurs. Ctrl+C pour arreter.\n");
    while (keep_running) {
        int32_t c1 = enc1.count;
        int32_t c2 = enc2.count;
        double cps1 = enc1.speed_cps;
        double rpm1 = enc1.speed_rpm;
        double cps2 = enc2.speed_cps;
        double rpm2 = enc2.speed_rpm;

        printf("\rENC1 pos=%ld | v=%.2f counts/s | %.2f RPM || ENC2 pos=%ld | v=%.2f counts/s | %.2f RPM",
               (long)c1, cps1, rpm1, (long)c2, cps2, rpm2);
        fflush(stdout);
        usleep(100000); // affichage 10 Hz
    }

    pthread_join(speed_thread, NULL);
    printf("\nFinal ENC1=%ld | ENC2=%ld\n", (long)enc1.count, (long)enc2.count);
    return 0;
}