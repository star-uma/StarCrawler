/*
 * config.h — StarCrawler ESP32 para ROS 2
 * =======================================
 * El ESP32 es la capa de tiempo real: pulsos de los steppers, lazo de posicion
 * con los encoders y bus CAN. Toda la inteligencia (mando, navegacion,
 * telemetria) vive en el PC de a bordo y habla por USB-serie.
 *
 * Sin WiFi ni Bluetooth: se compila con el core esp32 estandar.
 * El hardware comun esta en hw_comun.h, de firmware/libraries/StarCrawlerHW.
 * Aqui la compensacion de traccion en elevacion se SUMA a la velocidad de
 * conduccion: el PC puede pedir avanzar y bascular a la vez.
 */
#pragma once

#include <hw_comun.h>

/* ═══ ENLACE SERIE CON EL PC ══════════════════════════════════════════════ */

#define SERIE_BAUDIOS 921600
/* Sin consigna valida en este tiempo -> estado seguro. Mas apretado que los
 * 500 ms del enlace WiFi porque el USB es local y va a 50 tramas/s. */
#define WATCHDOG_TIMEOUT_MS 300

#define TELEMETRIA_CADA_N_CICLOS  2   /* tramas STATE al PC a 50 Hz */

/* Texto por el mismo puerto que las tramas binarias: solo el banner de
 * arranque (el parser del PC descarta lo que no sean tramas validas).
 * Dejar a 0 salvo para depurar a mano con un monitor serie. */
#define DEBUG_TEXTO 0
