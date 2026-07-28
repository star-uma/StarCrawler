/*
 * sensors.h
 * =========
 * Sensores I2C del robot:
 *   - 4x AS5600 (ángulo de cada oruga) tras el multiplexor TCA9548A
 *   - MPU9250 (IMU para el nivelado automático, modo 5)
 *
 * La IMU estaba conectada al MKR en el diseño original; en la arquitectura
 * unificada cuelga del mismo bus I2C del ESP32 (dirección 0x68, sin
 * conflicto con el TCA9548A en 0x70).
 */
#pragma once

#include <stdint.h>
#include "control_core.h"

/* ── Encoders AS5600 ────────────────────────────────────────────────────── */

void encoders_init();

/* Lee el ángulo de la oruga 'idx' {FR,FL,RR,RL} en grados (con offset).
 * Lectura de los dos bytes en una única transacción I2C (el original leía
 * byte alto y bajo por separado y podía mezclar dos muestras distintas).
 * Devuelve false si el sensor no responde. */
bool encoders_leer(int idx, float *angDeg);

/* ── IMU MPU9250 ────────────────────────────────────────────────────────── */

/* Despierta la IMU y configura rangos (±2 g, ±250 dps).
 * Devuelve false si no está presente en el bus. */
bool imu_init();

/* Calibración de la deriva del giroscopio (robot quieto). Bloqueante ~1 s. */
void imu_calibrarGyro();

/* Lee la IMU, actualiza el filtro complementario y devuelve roll/pitch en
 * grados según el criterio del TFG (fig. 7.40). false si falla la lectura. */
bool imu_actualizar(cc_FiltroActitud *filtro, float dt);
