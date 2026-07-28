/*
 * config.h — StarCrawler STANDALONE (sin PC)
 * ==========================================
 * Configuración del firmware autónomo: el mando Xbox se conecta por
 * Bluetooth directamente al ESP32 (Bluepad32). Sin PC, sin WiFi.
 * Solo tracción (RMD-X8 por CAN) y elevación (DM542 + AS5600). Sin IMU.
 */
#pragma once

/* ═══ BACKEND CAN ═════════════════════════════════════════════════════════ */
#define CAN_BACKEND_MCP2515 1
#define CAN_BACKEND_TWAI    2

#ifndef CAN_BACKEND
#define CAN_BACKEND CAN_BACKEND_MCP2515
#endif

/* ═══ MANDO (sustituye a la sección RED de las otras variantes) ═══════════ */

/* Ganancia del joystick: [-1,1] -> [-30,30] dps (GAIN del Python) */
#define GANANCIA_JOYSTICK 30.0f

/* Zona muerta de los sticks (DEADZONE del Python) */
#define ZONA_MUERTA 0.08f

/* Umbral de los gatillos para considerarlos pulsados (0..1) */
#define UMBRAL_TRIGGER 0.3f

/* Fondo de escala de los ejes de Bluepad32 (aprox. -512..511) */
#define FONDO_ESCALA_EJE 512.0f

/* ═══ TEMPORIZACIÓN ═══════════════════════════════════════════════════════ */

#define CICLO_CONTROL_MS          10   /* lazo de control a 100 Hz */
/* Watchdog: sin datos nuevos del mando en este tiempo -> parada segura.
 * (La desconexión Bluetooth ya dispara la seguridad al instante; esto cubre
 * un mando colgado que deja de refrescar.) */
#define WATCHDOG_TIMEOUT_MS       500
#define ENVIO_CAN_CADA_N_CICLOS   2    /* tramas de tracción a 50 Hz */
#define ESTADO_SERIE_CADA_N_CICLOS 100 /* línea de estado por serie a 1 Hz */

/* ═══ TRACCIÓN (RMD-X8 por CAN) ═══════════════════════════════════════════ */

#define CAN_ID_FL 0x141
#define CAN_ID_FR 0x142
#define CAN_ID_RR 0x143
#define CAN_ID_RL 0x144

#define VEL_MAX_DPS          40.0f
#define RATE_LIMIT_DPS_CICLO 4.0f
#define CAN_INTER_FRAME_US   250

/* ═══ COMPENSACIÓN DE TRACCIÓN EN MODOS DE ELEVACIÓN ══════════════════════
 * Ver docs/arquitectura_esp32_unificada.md (regla del modelo Simulink):
 * subiendo -> +5 º/s, bajando -> -5 º/s, parada -> 0.
 */
#define COMPENSACION_TRACCION 1
#define COMPENSACION_DPS      5.0f
#define TABLA_SIGNO_COMPENSACION { +1.0f, -1.0f, +1.0f, -1.0f }

/* ═══ PINES (ESP32 DevKit V1, 30 pines) ═══════════════════════════════════ */

#define PIN_SPI_SCK  18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_CAN_CS   5
#define PIN_CAN_INT  35

#define PIN_TWAI_TX  5
#define PIN_TWAI_RX  35

#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

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
