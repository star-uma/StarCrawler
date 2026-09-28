/*
 * hw_comun.h
 * ==========
 * Configuración del hardware del robot, común a todas las variantes del
 * firmware: bus CAN, motores, pines, cadena de elevación y encoders.
 *
 * Lo que sale de la calibración (offsets, tablas de signo y de sentido) se
 * apunta aquí una sola vez. Lo propio de cada variante (red, enlace serie,
 * IMU, watchdog) va en su config.h, que incluye este fichero.
 */
#pragma once

/* ═══ BACKEND CAN ═════════════════════════════════════════════════════════
 * CAN_BACKEND_TWAI (por defecto): controlador CAN interno del ESP32 (TWAI)
 *   + transceptor externo. Es el hardware del robot (issue #7).
 * CAN_BACKEND_MCP2515: módulo externo MCP2515 por SPI (cristal de 16 MHz
 *   OBLIGATORIO para 1 Mbps). Se elige con -DCAN_BACKEND=1.
 */
#define CAN_BACKEND_MCP2515 1
#define CAN_BACKEND_TWAI    2

#ifndef CAN_BACKEND
#define CAN_BACKEND CAN_BACKEND_TWAI
#endif

/* ═══ TEMPORIZACIÓN DEL LAZO ══════════════════════════════════════════════ */

#define CICLO_CONTROL_MS          10   /* lazo de control a 100 Hz */
#define ENVIO_CAN_CADA_N_CICLOS   2    /* tramas de tracción a 50 Hz */

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
 * Con ángulo <180º el original usaba 5 º/s constante; con >180º, velocidad
 * variable según  w = d·sin(a)·w_stepper / R2  (TFG, ec. 7.7). Aquí se usa
 * constante en todo el rango como aproximación: falta fijar el origen de
 * "a" y qué radio es R2. Ver docs/cadena_de_elevacion.md e issue #6.
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

/* I2C (TCA9548A + AS5600 x4, y la MPU9250 en la variante con IMU) */
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

/* ═══ CADENA DE ELEVACIÓN (steppers) ══════════════════════════════════════ */

/* Semiperiodo del tren de pulsos STEP (µs). 1200 µs = ~417 pasos/s,
 * mismo valor que el timer del firmware original. */
#define SEMIPERIODO_STEP_US 1200

/* Rampa de aceleración: arrancar de golpe hace perder pasos (inercia del
 * brazo reflejada a través de la reductora 1:80). El semiperiodo baja de
 * SEMIPERIODO_ARRANQUE_US a SEMIPERIODO_STEP_US restando
 * RAMPA_DECREMENTO_TICKS en cada pulso completo (~0.2 s con estos valores).
 * TICK_ISR_US es el periodo base del temporizador y debe dividir a los dos
 * semiperiodos; a 50 µs la ISR corre a 20 kHz. */
#define RAMPA_ACTIVA             1
#define SEMIPERIODO_ARRANQUE_US  4800
#define RAMPA_DECREMENTO_TICKS   2
#define TICK_ISR_US              50

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
