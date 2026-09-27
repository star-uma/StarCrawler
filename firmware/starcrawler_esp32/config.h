/*
 * config.h
 * ========
 * Configuración del firmware unificado StarCrawler (ESP32), modos 1-5 con IMU.
 * El hardware común (CAN, pines, steppers, encoders) está en hw_comun.h, de
 * firmware/libraries/StarCrawlerHW; aquí solo lo propio de esta variante.
 */
#pragma once

#include <hw_comun.h>

/* ═══ RED ═════════════════════════════════════════════════════════════════ */

#define RED_SSID "Horu"
#define RED_PASS "Horu27T-I"

/* El ESP32 toma la IP que tenía el MKR: StarCrawlerXbox.py no cambia.
 * IMPORTANTE: el MKR debe estar fuera de la red (apagado o sin firmware). */
#define USAR_IP_ESTATICA 1
#define IP_ROBOT      192, 168, 10, 101
#define IP_PUERTA     192, 168, 10, 1
#define IP_MASCARA    255, 255, 255, 0

#define PUERTO_ROBOT       8885  /* recepción de datagramas del PC */
#define PUERTO_TELEMETRIA  8886  /* envío de telemetría hacia el PC */

/* ═══ TEMPORIZACIÓN ═══════════════════════════════════════════════════════ */

#define WATCHDOG_TIMEOUT_MS       500  /* igual que el MKR original */
#define TELEMETRIA_CADA_N_CICLOS  10   /* telemetría al PC a 10 Hz */

/* ═══ IMU MPU9250 (antes en el MKR, ahora en el bus I2C del ESP32) ════════ */

#define DIR_MPU9250 0x68

#define LIMITE_NIVELADO_DEG 2.0f   /* "limite = 2" del Chart del TFG */
#define ALPHA_FILTRO        0.98f  /* filtro complementario */
#define MUESTRAS_CAL_GYRO   500    /* calibración de deriva al arrancar */

/* Signos de roll/pitch según el montaje de la IMU (fig. 7.40 del TFG:
 * roll + = inclinado a la derecha; pitch + = inclinado hacia atrás).
 * ⚠ VERIFICAR EN HARDWARE antes de usar el modo 5 con el robot en el suelo. */
#define SIGNO_ROLL  (+1.0f)
#define SIGNO_PITCH (+1.0f)
