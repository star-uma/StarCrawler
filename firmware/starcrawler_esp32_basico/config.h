/*
 * config.h — StarCrawler BÁSICO (sin IMU)
 * ========================================
 * Solo tracción (RMD-X8 por CAN) y elevación (steppers DM542 + encoders
 * AS5600), sin IMU ni modo 5. El hardware común está en hw_comun.h, de
 * firmware/libraries/StarCrawlerHW; aquí solo lo propio de esta variante.
 */
#pragma once

#include <hw_comun.h>

/* ═══ RED ═════════════════════════════════════════════════════════════════ */

#define RED_SSID "Horu"
#define RED_PASS "Horu27T-I"

/* El ESP32 toma la IP que tenía el MKR: StarCrawlerXbox.py no cambia.
 * IMPORTANTE: el MKR debe estar fuera de la red. */
#define USAR_IP_ESTATICA 1
#define IP_ROBOT      192, 168, 10, 101
#define IP_PUERTA     192, 168, 10, 1
#define IP_MASCARA    255, 255, 255, 0

#define PUERTO_ROBOT       8885
#define PUERTO_TELEMETRIA  8886

/* ═══ TEMPORIZACIÓN ═══════════════════════════════════════════════════════ */

#define WATCHDOG_TIMEOUT_MS       500
#define TELEMETRIA_CADA_N_CICLOS  10   /* telemetría al PC a 10 Hz */
