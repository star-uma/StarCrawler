/*
 * config.h
 * ========
 * Configuración central del firmware unificado StarCrawler (ESP32).
 * Todos los pines, direcciones, umbrales y parámetros viven aquí.
 */
#pragma once

/* ═══ BACKEND CAN ═════════════════════════════════════════════════════════
 * CAN_BACKEND_MCP2515: módulo externo MCP2515 por SPI (cristal de 16 MHz
 *   OBLIGATORIO para 1 Mbps; con cristal de 8 MHz no se alcanza).
 * CAN_BACKEND_TWAI: controlador CAN interno del ESP32 (TWAI) + transceptor
 *   externo de 3.3 V (p. ej. SN65HVD230). Recomendado a futuro: libera SPI.
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
 * IMPORTANTE: el MKR debe estar fuera de la red (apagado o sin firmware). */
#define USAR_IP_ESTATICA 1
#define IP_ROBOT      192, 168, 10, 101
#define IP_PUERTA     192, 168, 10, 1
#define IP_MASCARA    255, 255, 255, 0

#define PUERTO_ROBOT       8885  /* recepción de datagramas del PC */
#define PUERTO_TELEMETRIA  8886  /* envío de telemetría hacia el PC */

/* ═══ TEMPORIZACIÓN ═══════════════════════════════════════════════════════ */

#define CICLO_CONTROL_MS          10   /* lazo de control a 100 Hz */
#define WATCHDOG_TIMEOUT_MS       500  /* igual que el MKR original */
#define ENVIO_CAN_CADA_N_CICLOS   2    /* tramas de tracción a 50 Hz */
#define TELEMETRIA_CADA_N_CICLOS  10   /* telemetría al PC a 10 Hz */

/* ═══ TRACCIÓN (RMD-X8 por CAN) ═══════════════════════════════════════════ */

#define CAN_ID_FL 0x141
#define CAN_ID_FR 0x142
#define CAN_ID_RR 0x143
#define CAN_ID_RL 0x144

#define VEL_MAX_DPS          40.0f
/* El MKR limitaba 8 dps por paquete (50 Hz). A 100 Hz son 4 dps por ciclo
 * para mantener la misma rampa (~250 ms de 0 a 40 dps). */
#define RATE_LIMIT_DPS_CICLO 4.0f
#define CAN_INTER_FRAME_US   250   /* fix del fallo FL/0x141 del original */

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

/* ═══ PINES ═══════════════════════════════════════════════════════════════
 * ESP32 DevKit V1 (30 pines). Presupuesto completo — quedan libres: GPIO0,
 * GPIO12 (strapping, evitados a propósito) y los input-only 34/36/39.
 *
 * NOTA sobre el original: los pines del TFG (18, 19, 5...) chocaban con el
 * bus SPI que ahora necesita el MCP2515, por eso el mapa cambia.
 */

/* SPI para MCP2515 (VSPI) */
#define PIN_SPI_SCK  18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_CAN_CS   5
#define PIN_CAN_INT  35  /* input-only; sin usar en modo polling */

/* TWAI (solo si CAN_BACKEND_TWAI): reutiliza las posiciones CS/INT */
#define PIN_TWAI_TX  5
#define PIN_TWAI_RX  35

/* I2C (TCA9548A + AS5600 x4 + MPU9250) */
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

/* Steppers DM542 — orden {FR, FL, RR, RL} como el firmware original */
#define PINES_STEP { 25, 26, 27, 32 }
#define PINES_DIR  { 33, 13, 14, 15 }
#define PINES_ENA  {  4, 16, 17,  2 }

/* Tablas de sentido del original: nivel del pin DIR para que el ángulo
 * del encoder AUMENTE ("sentido horario" respecto de F.R.). */
#define TABLA_DIR_HORARIO     { 0, 1, 1, 0 }
#define TABLA_DIR_ANTIHORARIO { 1, 0, 0, 1 }

/* Semiperiodo del tren de pulsos STEP (µs). 1200 µs = ~417 pasos/s,
 * mismo valor que el timer del firmware original. */
#define SEMIPERIODO_STEP_US 1200

/* Al parar un motor, replicar el original: ENA+ en alto = driver
 * deshabilitado (sin par de retención; aguanta la reductora 1:80).
 * Poner a 0 para mantener par de retención al parar. */
#define PARADA_LIBERA_DRIVER 1

/* ═══ ENCODERS (AS5600 tras TCA9548A) ═════════════════════════════════════ */

#define DIR_TCA9548A 0x70
#define DIR_AS5600   0x36

/* Offsets de montaje por oruga {FR, FL, RR, RL} — valores del original */
#define OFFSETS_ENCODER { -2.0f, -5.0f, 16.0f, -15.0f }

/* Histéresis del control de posición (el original usaba 1º único) */
#define UMBRAL_ARRANQUE_DEG 1.0f
#define UMBRAL_PARADA_DEG   0.5f

/* Límites software de recorrido de las orugas. El rango mecánico del TFG
 * es 90..270º (vertical arriba/abajo); margen de 5º. */
#define LIMITES_SOFTWARE_ACTIVOS 1
#define ANGULO_MIN_DEG 85.0f
#define ANGULO_MAX_DEG 275.0f

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
