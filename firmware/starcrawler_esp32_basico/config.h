/*
 * config.h — StarCrawler BÁSICO (sin IMU)
 * ========================================
 * Configuración del firmware básico: solo tracción (RMD-X8 por CAN) y
 * elevación (steppers DM542 + encoders AS5600). Sin IMU, sin modo 5.
 */
#pragma once

/* ═══ BACKEND CAN ═════════════════════════════════════════════════════════
 * CAN_BACKEND_MCP2515: módulo externo MCP2515 por SPI (cristal de 16 MHz
 *   OBLIGATORIO para 1 Mbps).
 * CAN_BACKEND_TWAI: controlador CAN interno del ESP32 + transceptor 3.3 V.
 */
#define CAN_BACKEND_MCP2515 1
#define CAN_BACKEND_TWAI    2

#ifndef CAN_BACKEND
#define CAN_BACKEND CAN_BACKEND_MCP2515
#endif

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

#define CICLO_CONTROL_MS          10   /* lazo de control a 100 Hz */
#define WATCHDOG_TIMEOUT_MS       500
#define ENVIO_CAN_CADA_N_CICLOS   2    /* tramas de tracción a 50 Hz */
#define TELEMETRIA_CADA_N_CICLOS  10   /* telemetría al PC a 10 Hz */

/* ═══ TRACCIÓN (RMD-X8 por CAN) ═══════════════════════════════════════════ */

#define CAN_ID_FL 0x141
#define CAN_ID_FR 0x142
#define CAN_ID_RR 0x143
#define CAN_ID_RL 0x144

#define VEL_MAX_DPS          40.0f
#define RATE_LIMIT_DPS_CICLO 4.0f   /* misma rampa que el MKR (8 dps a 50 Hz) */
#define CAN_INTER_FRAME_US   250    /* fix del fallo FL/0x141 del original */

/* ═══ COMPENSACIÓN DE TRACCIÓN EN MODOS DE ELEVACIÓN ══════════════════════
 * Del modelo Control_MKR_Completo.slx: mientras una oruga bascula, su RMD
 * gira para que la banda no arrastre sobre el suelo:
 *   - oruga subiendo (ángulo del encoder disminuye)  -> +COMPENSACION_DPS (s.h)
 *   - oruga bajando  (ángulo del encoder aumenta)    -> -COMPENSACION_DPS (s.ah)
 *   - oruga parada                                    -> velocidad 0 (frenado)
 * El modelo original usaba 5 º/s constante con ángulo <180º y una velocidad
 * variable con ángulo >180º cuya fórmula queda por extraer del .slx: aquí se
 * usa constante en todo el rango como aproximación.
 */
#define COMPENSACION_TRACCION 1
#define COMPENSACION_DPS      5.0f

/* Signo eléctrico por motor {FR, FL, RR, RL}: el lado izquierdo va invertido
 * como en tracción. ⚠ VERIFICAR EN HARDWARE con el robot sobre tacos. */
#define TABLA_SIGNO_COMPENSACION { +1.0f, -1.0f, +1.0f, -1.0f }

/* ═══ PINES (ESP32 DevKit V1, 30 pines) ═══════════════════════════════════ */

/* SPI para MCP2515 (VSPI) */
#define PIN_SPI_SCK  18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_CAN_CS   5
#define PIN_CAN_INT  35  /* input-only; sin usar en modo polling */

/* TWAI (solo si CAN_BACKEND_TWAI) */
#define PIN_TWAI_TX  5
#define PIN_TWAI_RX  35

/* I2C (TCA9548A + AS5600 x4) */
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

/* Steppers DM542 — orden {FR, FL, RR, RL} */
#define PINES_STEP { 25, 26, 27, 32 }
#define PINES_DIR  { 33, 13, 14, 15 }
#define PINES_ENA  {  4, 16, 17,  2 }

/* Nivel del pin DIR para que el ángulo del encoder AUMENTE */
#define TABLA_DIR_HORARIO     { 0, 1, 1, 0 }
#define TABLA_DIR_ANTIHORARIO { 1, 0, 0, 1 }

#define SEMIPERIODO_STEP_US 1200  /* ~417 pasos/s, como el original */

/* Al parar: ENA+ alto = driver deshabilitado (como el original) */
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
