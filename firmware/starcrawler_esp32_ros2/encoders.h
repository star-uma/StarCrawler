/*
 * encoders.h — StarCrawler BÁSICO (sin IMU)
 * =========================================
 * 4x AS5600 (ángulo de cada oruga) tras el multiplexor TCA9548A.
 */
#pragma once

#include <stdint.h>
#include "control_core.h"

void encoders_init();

/* Lee el ángulo de la oruga 'idx' {FR,FL,RR,RL} en grados (con offset).
 * Lectura de los dos bytes en una única transacción I2C.
 * Devuelve false si el sensor no responde. */
bool encoders_leer(int idx, float *angDeg);
