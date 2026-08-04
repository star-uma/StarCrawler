/*
 * config.h — StarCrawler ESP32 para ROS 2
 * =======================================
 * El ESP32 es la capa de tiempo real: pulsos de los steppers, lazo de posicion
 * con los encoders y bus CAN. Toda la inteligencia (mando, navegacion,
 * telemetria) vive en el PC de a bordo y habla por USB-serie.
 *
 * Sin WiFi ni Bluetooth: se compila con el core esp32 estandar.
 */
#pragma once

/* ═══ BACKEND CAN ═════════════════════════════════════════════════════════ */
#define CAN_BACKEND_MCP2515 1
#define CAN_BACKEND_TWAI    2

#ifndef CAN_BACKEND
#define CAN_BACKEND CAN_BACKEND_MCP2515
#endif

/* ═══ ENLACE SERIE CON EL PC ══════════════════════════════════════════════ */

#define SERIE_BAUDIOS 921600
/* Sin consigna valida en este tiempo -> estado seguro. Mas apretado que los
 * 500 ms del enlace WiFi porque el USB es local y va a 50 tramas/s. */
#define WATCHDOG_TIMEOUT_MS 300

#define CICLO_CONTROL_MS          10  /* lazo de control a 100 Hz */
#define ENVIO_CAN_CADA_N_CICLOS   2   /* tramas de traccion a 50 Hz */
#define TELEMETRIA_CADA_N_CICLOS  2   /* tramas STATE al PC a 50 Hz */

/* Texto por el mismo puerto que las tramas binarias: solo el banner de
 * arranque (el parser del PC descarta lo que no sean tramas validas).
 * Dejar a 0 salvo para depurar a mano con un monitor serie. */
#define DEBUG_TEXTO 0

/* ═══ TRACCION (RMD-X8 por CAN) ═══════════════════════════════════════════ */

#define CAN_ID_FL 0x141
#define CAN_ID_FR 0x142
#define CAN_ID_RR 0x143
#define CAN_ID_RL 0x144

#define VEL_MAX_DPS          40.0f
#define RATE_LIMIT_DPS_CICLO 4.0f
#define CAN_INTER_FRAME_US   250

/* ═══ COMPENSACION DE TRACCION EN ELEVACION ═══════════════════════════════
 * Del modelo Control_MKR_Completo.slx: mientras una oruga bascula, su RMD
 * gira para que la banda no arrastre. Aqui se SUMA a la velocidad de
 * conduccion (el PC puede pedir avanzar y bascular a la vez).
 * TABLA_SIGNO: pendiente de verificar con el robot sobre tacos.
 */
#define COMPENSACION_TRACCION 1
#define COMPENSACION_DPS      5.0f
#define TABLA_SIGNO_COMPENSACION { +1.0f, -1.0f, +1.0f, -1.0f }

/* ═══ PINES (ESP32 DevKit V1) ═════════════════════════════════════════════ */

#define PIN_SPI_SCK  18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_CAN_CS   5
#define PIN_CAN_INT  35

#define PIN_TWAI_TX  5
#define PIN_TWAI_RX  35

#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

/* Steppers DM542 — orden {FR, FL, RR, RL} */
#define PINES_STEP { 25, 26, 27, 32 }
#define PINES_DIR  { 33, 13, 14, 15 }
#define PINES_ENA  {  4, 16, 17,  2 }

#define TABLA_DIR_HORARIO     { 0, 1, 1, 0 }
#define TABLA_DIR_ANTIHORARIO { 1, 0, 0, 1 }

#define SEMIPERIODO_STEP_US 1200
#define PARADA_LIBERA_DRIVER 1

/* ═══ ENCODERS (AS5600 tras TCA9548A) ═════════════════════════════════════ */

#define DIR_TCA9548A 0x70
#define DIR_AS5600   0x36

#define OFFSETS_ENCODER { -2.0f, -5.0f, 16.0f, -15.0f }

#define UMBRAL_ARRANQUE_DEG 1.0f
#define UMBRAL_PARADA_DEG   0.5f

#define LIMITES_SOFTWARE_ACTIVOS 1
#define ANGULO_MIN_DEG 85.0f
#define ANGULO_MAX_DEG 275.0f
