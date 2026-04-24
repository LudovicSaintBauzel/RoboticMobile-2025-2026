// encoder.c — Lecture position + vitesse d'un encodeur quadrature (2 GPIOs)
//
// Compilation :
//   cc -std=c11 -O2 -Wall -Wextra encoder.c -o encoder -lwiringPi -pthread
//
// Usage :
//   ./encoder              -> GPIO_A=18, GPIO_B=19 (defaut)
//   ./encoder 16 17        -> GPIO_A=16, GPIO_B=17

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <wiringPi.h>

#define DEFAULT_GPIO_A 18
#define DEFAULT_GPIO_B 19
#define ENCODER_CPR    1440.0

static volatile sig_atomic_t keep_running = 1;

static volatile int32_t enc_count      = 0;
static volatile uint8_t enc_last_state = 0;
static volatile double  enc_speed_cps  = 0.0;
static volatile double  enc_speed_rpm  = 0.0;
static volatile double  enc_angular    = 0.0;

static int gpio_a;
static int gpio_b;

static const int8_t qdec_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
     -1,  0,  0,  1,
     0,  1, -1,  0
};
/* usleep() est marque obsolete par POSIX.1-2008 ; nanosleep est C11-safe */
static void sleep_us(unsigned long us) {
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000UL);
    ts.tv_nsec = (long)((us % 1000000UL) * 1000UL);
    nanosleep(&ts, NULL);
}
static void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

static void encoder_isr(void) {
    uint8_t a = (uint8_t)digitalRead(gpio_a);
    uint8_t b = (uint8_t)digitalRead(gpio_b);
    uint8_t new_state = (uint8_t)((a << 1) | b);
    uint8_t idx = (uint8_t)((enc_last_state << 2) | new_state);
    enc_count += qdec_table[idx];
    enc_last_state = new_state;
}

static double elapsed_s(const struct timespec *t0, const struct timespec *t1) {
    return (double)(t1->tv_sec - t0->tv_sec)
         + (double)(t1->tv_nsec - t0->tv_nsec) / 1e9;
}

static void *speed_thread_fn(void *arg) {
    (void)arg;
    int32_t prev = enc_count;
    struct timespec t_prev, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_prev);

    while (keep_running) {
        sleep_us(100000);  /* 10 Hz */

        int32_t now = enc_count;
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double dt = elapsed_s(&t_prev, &t_now);

        if (dt > 0.0) {
            enc_speed_cps = (double)(now - prev) / dt;
            enc_speed_rpm = (enc_speed_cps / ENCODER_CPR) * 60.0;
            enc_angular = (double)enc_count * 360.0 / ENCODER_CPR;
        }

        prev   = now;
        t_prev = t_now;
    }

    return NULL;
}

int main(int argc, char *argv[]) {
    gpio_a = DEFAULT_GPIO_A;
    gpio_b = DEFAULT_GPIO_B;

    if (argc == 3) {
        gpio_a = atoi(argv[1]);
        gpio_b = atoi(argv[2]);
    } else if (argc != 1) {
        fprintf(stderr, "Usage : %s [gpio_a gpio_b]\n", argv[0]);
        return 1;
    }

    signal(SIGINT, handle_sigint);

    if (wiringPiSetupGpio() < 0) {
        fprintf(stderr, "Erreur : wiringPiSetupGpio\n");
        return 1;
    }

    pinMode(gpio_a, INPUT);
    pinMode(gpio_b, INPUT);
    pullUpDnControl(gpio_a, PUD_UP);
    pullUpDnControl(gpio_b, PUD_UP);

    enc_last_state = (uint8_t)((digitalRead(gpio_a) << 1) | digitalRead(gpio_b));

    if (wiringPiISR(gpio_a, INT_EDGE_BOTH, encoder_isr) < 0 ||
        wiringPiISR(gpio_b, INT_EDGE_BOTH, encoder_isr) < 0) {
        fprintf(stderr, "Erreur : wiringPiISR\n");
        return 1;
    }

    pthread_t speed_t;
    if (pthread_create(&speed_t, NULL, speed_thread_fn, NULL) != 0) {
        fprintf(stderr, "Erreur : pthread_create\n");
        return 1;
    }

    printf("Encodeur sur GPIO_A=%d, GPIO_B=%d (%d CPR). Ctrl+C pour arreter.\n",
           gpio_a, gpio_b, (int)ENCODER_CPR);

    while (keep_running) {
        printf("\rpos=%ld | %+.2f counts/s | %+.2f RPM  | %+.2f deg",
               (long)enc_count, enc_speed_cps, enc_speed_rpm, enc_angular);
        fflush(stdout);
        sleep_us(100000);  /* 10 Hz */
    }

    pthread_join(speed_t, NULL);
    printf("\nFinal pos=%ld\n", (long)enc_count);
    return 0;
}
