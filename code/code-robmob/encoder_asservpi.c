// encoder_asservpi.c — Encodeurs quadrature + calcul vitesse + PID moteur I2C
// Standard : C11 + pthread
//
// ── Compilation mode virtuel (macOS / test sans GPIO) ──────────────────────
//   cc -std=c11 -O2 -Wall -Wextra encoder_asservpi.c -o encoder_asservpi_c -pthread
//
// ── Compilation mode Raspberry Pi ───────────────────────────────────────────
//   cc -std=c11 -O2 -Wall -Wextra -DRPI encoder_asservpi.c -o encoder_asservpi_c \
//      -lwiringPi -pthread
//
// ── Usage ───────────────────────────────────────────────────────────────────
//   ./encoder_asservpi_c              -> M1=M2=80 RPM (avant)
//   ./encoder_asservpi_c 60           -> M1=M2=60 RPM (avant)
//   ./encoder_asservpi_c 80 -60       -> M1=80 RPM avant, M2=60 RPM arriere
//   RPM positif = avant (CW), RPM negatif = arriere (CCW)

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef RPI
#  include <fcntl.h>
#  include <linux/i2c-dev.h>
#  include <sys/ioctl.h>
#  include <wiringPi.h>
#endif

#define ENC1_A 18
#define ENC1_B 19
#define ENC2_A 16
#define ENC2_B 17

#define ENCODER_CPR 1440.0
#define I2C_ADDR 0x0F
#define PID_PERIOD_MS 50U
#define I2C_REG_SPEED 0x82
#define I2C_REG_DIR 0xAA
#define DEFAULT_RPM 80.0

#define PID_KP 0.9
#define PID_KI 0.2
#define PID_KD 0.02

static const int8_t qdec_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

typedef struct {
    atomic_int_fast32_t count;
    atomic_uint_fast8_t last_state;
    atomic_uint_fast64_t speed_cps_bits;
    atomic_uint_fast64_t speed_rpm_bits;
} encoder_t;

typedef struct {
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
    double out_min;
    double out_max;
} pid_controller_t;

typedef struct {
    encoder_t *enc1;
    encoder_t *enc2;
} speed_thread_args_t;

#ifdef RPI
typedef struct {
    encoder_t *enc1;
    encoder_t *enc2;
    double target_rpm1;
    double target_rpm2;
} pid_thread_args_t;
#endif

static atomic_bool g_running = false;
static encoder_t g_enc1;
static encoder_t g_enc2;

_Static_assert(sizeof(double) == sizeof(uint64_t), "double must be 64-bit IEEE-754");

static void atomic_store_f64(atomic_uint_fast64_t *dst, double value) {
    uint64_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    atomic_store_explicit(dst, bits, memory_order_relaxed);
}

static double atomic_load_f64(const atomic_uint_fast64_t *src) {
    uint64_t bits = atomic_load_explicit(src, memory_order_relaxed);
    double value = 0.0;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void encoder_init(encoder_t *enc) {
    atomic_init(&enc->count, 0);
    atomic_init(&enc->last_state, 0);
    atomic_init(&enc->speed_cps_bits, 0);
    atomic_init(&enc->speed_rpm_bits, 0);
}

static void encoder_set_speed(encoder_t *enc, double cps, double rpm) {
    atomic_store_f64(&enc->speed_cps_bits, cps);
    atomic_store_f64(&enc->speed_rpm_bits, rpm);
}

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void update_encoder_from_state(encoder_t *enc, uint8_t new_state) {
    uint8_t last = (uint8_t)atomic_load_explicit(&enc->last_state, memory_order_relaxed);
    uint8_t idx = (uint8_t)((last << 2) | new_state);
    atomic_fetch_add_explicit(&enc->count, qdec_table[idx], memory_order_relaxed);
    atomic_store_explicit(&enc->last_state, new_state, memory_order_relaxed);
}

static void print_status(const encoder_t *enc1, const encoder_t *enc2,
                         double target_rpm1, double target_rpm2) {
    int32_t pos1 = (int32_t)atomic_load_explicit(&enc1->count, memory_order_relaxed);
    int32_t pos2 = (int32_t)atomic_load_explicit(&enc2->count, memory_order_relaxed);
    double rpm1 = atomic_load_f64(&enc1->speed_rpm_bits);
    double rpm2 = atomic_load_f64(&enc2->speed_rpm_bits);

    printf("\rENC1 [cible=%+.0f RPM] pos=%ld | %+.2f RPM || ENC2 [cible=%+.0f RPM] pos=%ld | %+.2f RPM",
           target_rpm1, (long)pos1, rpm1, target_rpm2, (long)pos2, rpm2);
    fflush(stdout);
}

static void *speed_thread_fn(void *arg) {
    speed_thread_args_t *args = (speed_thread_args_t *)arg;
    int32_t prev1 = (int32_t)atomic_load_explicit(&args->enc1->count, memory_order_relaxed);
    int32_t prev2 = (int32_t)atomic_load_explicit(&args->enc2->count, memory_order_relaxed);
    double t_prev = monotonic_seconds();

    while (atomic_load_explicit(&g_running, memory_order_relaxed)) {
        usleep(100000);

        int32_t now1 = (int32_t)atomic_load_explicit(&args->enc1->count, memory_order_relaxed);
        int32_t now2 = (int32_t)atomic_load_explicit(&args->enc2->count, memory_order_relaxed);
        double t_now = monotonic_seconds();
        double dt = t_now - t_prev;

        if (dt > 0.0) {
            double cps1 = (double)(now1 - prev1) / dt;
            double cps2 = (double)(now2 - prev2) / dt;
            encoder_set_speed(args->enc1, cps1, cps1 / ENCODER_CPR * 60.0);
            encoder_set_speed(args->enc2, cps2, cps2 / ENCODER_CPR * 60.0);
        }

        prev1 = now1;
        prev2 = now2;
        t_prev = t_now;
    }

    return NULL;
}

static void handle_sigint(int sig) {
    (void)sig;
    atomic_store_explicit(&g_running, false, memory_order_relaxed);
}

static int parse_targets(int argc, char *argv[], double *target_rpm1, double *target_rpm2) {
    char *end = NULL;

    *target_rpm1 = DEFAULT_RPM;
    *target_rpm2 = DEFAULT_RPM;

    if (argc > 3) {
        fprintf(stderr, "Usage : %s [rpm_m1 [rpm_m2]]\n", argv[0]);
        return -1;
    }

    if (argc >= 2) {
        errno = 0;
        *target_rpm1 = strtod(argv[1], &end);
        if (errno != 0 || end == argv[1] || *end != '\0') {
            fprintf(stderr, "Erreur argument rpm_m1 : %s\n", argv[1]);
            return -1;
        }
        *target_rpm2 = *target_rpm1;
    }

    if (argc == 3) {
        errno = 0;
        *target_rpm2 = strtod(argv[2], &end);
        if (errno != 0 || end == argv[2] || *end != '\0') {
            fprintf(stderr, "Erreur argument rpm_m2 : %s\n", argv[2]);
            return -1;
        }
    }

    return 0;
}

#ifdef RPI

static double clamp_f64(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static double encoder_speed_rpm(const encoder_t *enc) {
    return atomic_load_f64(&enc->speed_rpm_bits);
}

static void pid_init(pid_controller_t *pid, double kp, double ki, double kd,
                     double out_min, double out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

static double pid_update(pid_controller_t *pid, double setpoint, double measured, double dt) {
    double error = setpoint - measured;
    double derivative = 0.0;
    double output;

    pid->integral += error * dt;
    if (dt > 0.0) {
        derivative = (error - pid->prev_error) / dt;
    }

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = clamp_f64(output, pid->out_min, pid->out_max);

    if (pid->ki != 0.0) {
        double i_max = pid->out_max / fabs(pid->ki);
        pid->integral = clamp_f64(pid->integral, -i_max, i_max);
    }

    pid->prev_error = error;
    return output;
}

static uint8_t saturate_u8(double value) {
    double rounded = round(value);
    if (rounded < 0.0) {
        return 0;
    }
    if (rounded > 255.0) {
        return 255;
    }
    return (uint8_t)rounded;
}

static uint8_t direction_byte(double rpm1, double rpm2) {
    uint8_t d1 = (rpm1 >= 0.0) ? 0x01u : 0x02u;
    uint8_t d2 = (rpm2 >= 0.0) ? 0x01u : 0x02u;
    return (uint8_t)(d1 | (uint8_t)(d2 << 2));
}

static void *pid_i2c_thread_fn(void *arg) {
    pid_thread_args_t *args = (pid_thread_args_t *)arg;
    pid_controller_t pid1;
    pid_controller_t pid2;
    double t_prev;
    uint8_t last_dir;
    int fd;

    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Erreur I2C : ouverture /dev/i2c-1 impossible\n");
        return NULL;
    }

    if (ioctl(fd, I2C_SLAVE, (long)I2C_ADDR) < 0) {
        fprintf(stderr, "Erreur I2C : selection adresse 0x%02X\n", I2C_ADDR);
        close(fd);
        return NULL;
    }

    {
        uint8_t dir_buf[3] = { I2C_REG_DIR, direction_byte(args->target_rpm1, args->target_rpm2), 0x01 };
        if (write(fd, dir_buf, sizeof(dir_buf)) != (ssize_t)sizeof(dir_buf)) {
            fprintf(stderr, "Erreur I2C write reg 0xAA (direction initiale)\n");
        }
    }

    pid_init(&pid1, PID_KP, PID_KI, PID_KD, 0.0, 255.0);
    pid_init(&pid2, PID_KP, PID_KI, PID_KD, 0.0, 255.0);
    t_prev = monotonic_seconds();
    last_dir = direction_byte(args->target_rpm1, args->target_rpm2);

    while (atomic_load_explicit(&g_running, memory_order_relaxed)) {
        double t_now = monotonic_seconds();
        double dt = t_now - t_prev;
        uint8_t cur_dir;
        uint8_t speed_buf[3];
        uint8_t dir_buf[3];
        uint8_t cmd1;
        uint8_t cmd2;

        t_prev = t_now;

        cmd1 = saturate_u8(pid_update(&pid1, fabs(args->target_rpm1), fabs(encoder_speed_rpm(args->enc1)), dt));
        cmd2 = saturate_u8(pid_update(&pid2, fabs(args->target_rpm2), fabs(encoder_speed_rpm(args->enc2)), dt));

        cur_dir = direction_byte(args->target_rpm1, args->target_rpm2);
        if (cur_dir != last_dir) {
            dir_buf[0] = I2C_REG_DIR;
            dir_buf[1] = cur_dir;
            dir_buf[2] = 0x01;
            if (write(fd, dir_buf, sizeof(dir_buf)) != (ssize_t)sizeof(dir_buf)) {
                fprintf(stderr, "Erreur I2C write reg 0xAA (direction)\n");
            }
            last_dir = cur_dir;
        }

        speed_buf[0] = I2C_REG_SPEED;
        speed_buf[1] = cmd1;
        speed_buf[2] = cmd2;
        if (write(fd, speed_buf, sizeof(speed_buf)) != (ssize_t)sizeof(speed_buf)) {
            fprintf(stderr, "Erreur I2C write reg 0x82\n");
        }

        usleep(PID_PERIOD_MS * 1000U);
    }

    {
        uint8_t stop_buf[3] = { I2C_REG_SPEED, 0x00, 0x00 };
        (void)write(fd, stop_buf, sizeof(stop_buf));
    }
    close(fd);
    return NULL;
}

static void encoder1_isr(void) {
    uint8_t new_state = (uint8_t)(((uint8_t)digitalRead(ENC1_A) << 1) | (uint8_t)digitalRead(ENC1_B));
    update_encoder_from_state(&g_enc1, new_state);
}

static void encoder2_isr(void) {
    uint8_t new_state = (uint8_t)(((uint8_t)digitalRead(ENC2_A) << 1) | (uint8_t)digitalRead(ENC2_B));
    update_encoder_from_state(&g_enc2, new_state);
}

static int run_rpi(double target_rpm1, double target_rpm2) {
    pthread_t speed_thread;
    pthread_t pid_thread;
    speed_thread_args_t speed_args;
    pid_thread_args_t pid_args;

    if (wiringPiSetupGpio() < 0) {
        fprintf(stderr, "Erreur : wiringPiSetupGpio() a echoue\n");
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

    atomic_store_explicit(&g_enc1.last_state,
                          (uint8_t)(((uint8_t)digitalRead(ENC1_A) << 1) | (uint8_t)digitalRead(ENC1_B)),
                          memory_order_relaxed);
    atomic_store_explicit(&g_enc2.last_state,
                          (uint8_t)(((uint8_t)digitalRead(ENC2_A) << 1) | (uint8_t)digitalRead(ENC2_B)),
                          memory_order_relaxed);

    if (wiringPiISR(ENC1_A, INT_EDGE_BOTH, encoder1_isr) < 0 ||
        wiringPiISR(ENC1_B, INT_EDGE_BOTH, encoder1_isr) < 0 ||
        wiringPiISR(ENC2_A, INT_EDGE_BOTH, encoder2_isr) < 0 ||
        wiringPiISR(ENC2_B, INT_EDGE_BOTH, encoder2_isr) < 0) {
        fprintf(stderr, "Erreur : wiringPiISR a echoue\n");
        return 1;
    }

    speed_args.enc1 = &g_enc1;
    speed_args.enc2 = &g_enc2;
    pid_args.enc1 = &g_enc1;
    pid_args.enc2 = &g_enc2;
    pid_args.target_rpm1 = target_rpm1;
    pid_args.target_rpm2 = target_rpm2;

    if (pthread_create(&speed_thread, NULL, speed_thread_fn, &speed_args) != 0) {
        fprintf(stderr, "Erreur : creation du thread vitesse impossible\n");
        return 1;
    }
    if (pthread_create(&pid_thread, NULL, pid_i2c_thread_fn, &pid_args) != 0) {
        fprintf(stderr, "Erreur : creation du thread PID impossible\n");
        atomic_store_explicit(&g_running, false, memory_order_relaxed);
        pthread_join(speed_thread, NULL);
        return 1;
    }

    printf("Comptage + vitesse (10 Hz) avec PID I2C.\n");
    printf("Consignes : M1=%+.0f RPM, M2=%+.0f RPM. Ctrl+C pour arreter.\n",
           target_rpm1, target_rpm2);

    while (atomic_load_explicit(&g_running, memory_order_relaxed)) {
        print_status(&g_enc1, &g_enc2, target_rpm1, target_rpm2);
        usleep(100000);
    }

    pthread_join(pid_thread, NULL);
    pthread_join(speed_thread, NULL);
    return 0;
}

#else

static int run_virtual(double target_rpm1, double target_rpm2) {
    static const uint8_t seq_forward[4] = { 0x00, 0x01, 0x03, 0x02 };
    static const uint8_t seq_reverse[4] = { 0x00, 0x02, 0x03, 0x01 };
    const uint8_t *seq1 = (target_rpm1 >= 0.0) ? seq_forward : seq_reverse;
    const uint8_t *seq2 = (target_rpm2 >= 0.0) ? seq_forward : seq_reverse;
    speed_thread_args_t speed_args;
    pthread_t speed_thread;
    size_t i1 = 0;
    size_t i2 = 0;

    speed_args.enc1 = &g_enc1;
    speed_args.enc2 = &g_enc2;

    if (pthread_create(&speed_thread, NULL, speed_thread_fn, &speed_args) != 0) {
        fprintf(stderr, "Erreur : creation du thread vitesse impossible\n");
        return 1;
    }

    printf("Mode virtuel actif. Consignes : M1=%+.0f RPM, M2=%+.0f RPM. Ctrl+C pour arreter.\n",
           target_rpm1, target_rpm2);

    while (atomic_load_explicit(&g_running, memory_order_relaxed)) {
        i1 = (i1 + 1U) % 4U;
        i2 = (i2 + 1U) % 4U;
        update_encoder_from_state(&g_enc1, seq1[i1]);
        update_encoder_from_state(&g_enc2, seq2[i2]);
        print_status(&g_enc1, &g_enc2, target_rpm1, target_rpm2);
        usleep(20000);
    }

    pthread_join(speed_thread, NULL);
    return 0;
}

#endif

int main(int argc, char *argv[]) {
    double target_rpm1;
    double target_rpm2;
    int rc;

    if (parse_targets(argc, argv, &target_rpm1, &target_rpm2) != 0) {
        fprintf(stderr,
                "RPM positif = avant (CW), negatif = arriere (CCW)\n"
                "Exemple : %s 80\n"
                "          %s 80 -60\n",
                argv[0], argv[0]);
        return 1;
    }

    signal(SIGINT, handle_sigint);
    atomic_store_explicit(&g_running, true, memory_order_relaxed);
    encoder_init(&g_enc1);
    encoder_init(&g_enc2);

#ifdef RPI
    rc = run_rpi(target_rpm1, target_rpm2);
#else
    rc = run_virtual(target_rpm1, target_rpm2);
#endif

    printf("\nFinal ENC1=%ld | ENC2=%ld\n",
           (long)atomic_load_explicit(&g_enc1.count, memory_order_relaxed),
           (long)atomic_load_explicit(&g_enc2.count, memory_order_relaxed));

    return rc;
}
