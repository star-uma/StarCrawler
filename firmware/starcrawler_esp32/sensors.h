/*
 * sensors.h
 * =========
 * IMU MPU9250 para el nivelado automatico (modo 5). Los encoders AS5600
 * estan en la biblioteca StarCrawlerHW (encoders.h), que se incluye aqui.
 *
 * La IMU estaba conectada al MKR en el diseño original; en la arquitectura
 * unificada cuelga del mismo bus I2C del ESP32 (dirección 0x68, sin
 * conflicto con el TCA9548A en 0x70).
 */
#pragma once

#include <stdint.h>
#include "control_core.h"

#include <encoders.h>   /* los 4 AS5600, en StarCrawlerHW */

/* ── IMU MPU9250 ────────────────────────────────────────────────────────── */

/* Despierta la IMU y configura rangos (±2 g, ±250 dps).
 * Devuelve false si no está presente en el bus. */
bool imu_init();

/* Calibración de la deriva del giroscopio (robot quieto). Bloqueante ~1 s. */
void imu_calibrarGyro();

/* Lee la IMU, actualiza el filtro complementario y devuelve roll/pitch en
 * grados según el criterio del TFG (fig. 7.40). false si falla la lectura. */
bool imu_actualizar(cc_FiltroActitud *filtro, float dt);
