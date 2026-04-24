// moteur.c — Commande moteur I2C (Grove I2C Motor Driver v1.3, adresse 0x0F)
//
// Compilation :
//   cc -std=c11 -O2 -Wall -Wextra moteur.c -o moteur -lwiringPi
//
// Usage :
//   ./moteur               -> intensite +255 (avant, pleine vitesse)
//   ./moteur 128            -> intensite +128 (avant)
//   ./moteur -100           -> intensite  100 (arriere)
//   Valeur entre -255 et 255. Signe = direction. Ctrl+C pour arreter.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>      /* close() */
#include <wiringPiI2C.h>

/* ══════════════════════════════════════════════════════════════════════════
 * Alternative POSIX I2C (sans wiringPi)
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Remplacer #include <wiringPiI2C.h> par :
 *   #include <fcntl.h>
 *   #include <sys/ioctl.h>
 *   #include <linux/i2c-dev.h>
 *
 * Ouverture du bus :
 *   int fd = open("/dev/i2c-1", O_RDWR);
 *   if (fd < 0) { perror("open /dev/i2c-1"); return 1; }
 *   if (ioctl(fd, I2C_SLAVE, (long)I2C_ADDR) < 0) {
 *       perror("ioctl I2C_SLAVE"); close(fd); return 1;
 *   }
 *
 * Ecriture d'un registre + 2 octets de donnees :
 *   static int i2c_write3(int fd, uint8_t reg, uint8_t b1, uint8_t b2) {
 *       uint8_t buf[3] = { reg, b1, b2 };
 *       return (write(fd, buf, 3) == 3) ? 0 : -1;
 *   }
 *
 * Utilisation (remplace les appels wiringPiI2C) :
 *   i2c_write3(fd, I2C_REG_DIR,   dir_byte, 0x01);   // direction
 *   i2c_write3(fd, I2C_REG_SPEED, speed,    0x00);    // vitesse M1
 *   i2c_write3(fd, I2C_REG_SPEED, 0x00,     0x00);    // arret
 *   close(fd);
 *
 * ══════════════════════════════════════════════════════════════════════════ */

#define I2C_ADDR          0x0F
#define I2C_REG_SPEED     0x82
#define I2C_REG_DIR       0xAA
#define DEFAULT_INTENSITY 255

/* Registre 0xAA — direction :
 *   bits [1:0] = M1 : 0b01 = CW (avant), 0b10 = CCW (arriere)
 *   bits [3:2] = M2 : 0b01 = CW (avant), 0b10 = CCW (arriere)
 * Suivi de 0x01 pour valider.
 *
 * Registre 0x82 — vitesse :
 *   [magnitude_M1, magnitude_M2]  (0..255 chacun) */



static volatile sig_atomic_t keep_running = 1;

static void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

static uint8_t direction_byte(int intensity) {
    /* M1 selon le signe, M2 arrete (CW par defaut) */
    uint8_t d1 = (intensity >= 0) ? 0x01u : 0x02u;
    uint8_t d2 = 0x01u;
    return (uint8_t)(d1 | (uint8_t)(d2 << 2));
}

int main(int argc, char *argv[]) {
    int intensity = DEFAULT_INTENSITY;
    int fd;
    uint8_t speed;
    uint8_t dir;

    if (argc == 2) {
        intensity = atoi(argv[1]);
        if (intensity < -255 || intensity > 255) {
            fprintf(stderr, "Intensite doit etre entre -255 et 255\n");
            return 1;
        }
    } else if (argc > 2) {
        fprintf(stderr,
                "Usage : %s [intensite]\n"
                "  intensite : -255..255 (defaut : %d)\n"
                "  Signe = direction (positif=avant, negatif=arriere)\n",
                argv[0], DEFAULT_INTENSITY);
        return 1;
    }

    signal(SIGINT, handle_sigint);

    /* ── Ouverture I2C (wiringPi) ──────────────────────────────────────── */
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd < 0) {
        fprintf(stderr, "Erreur : wiringPiI2CSetup(0x%02X)\n", I2C_ADDR);
        return 1;
    }

    speed = (uint8_t)abs(intensity);
    dir   = direction_byte(intensity);

    /* Direction : reg 0xAA, [dir_byte, 0x01]
     * wiringPiI2CWriteReg16 envoie : reg, val & 0xFF, (val >> 8) & 0xFF */
    wiringPiI2CWriteReg16(fd, I2C_REG_DIR, dir | (0x01 << 8));

    /* Vitesse : reg 0x82, [speed_M1, speed_M2=0] */
    wiringPiI2CWriteReg16(fd, I2C_REG_SPEED, speed | (0x00 << 8));

    printf("Moteur M1 : intensite=%d (magnitude=%u, dir=%s)\n",
           intensity, speed, (intensity >= 0) ? "avant" : "arriere");
    printf("Ctrl+C pour arreter le moteur.\n");

    while (keep_running) {
        sleep(1);  /* 1 s */
    }

    /* Arret moteur */
    wiringPiI2CWriteReg16(fd, I2C_REG_SPEED, 0x0000);
    printf("\nMoteur arrete.\n");
    close(fd);

    return 0;
}
