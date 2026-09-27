/*
 * encoders.h
 * ==========
 * 4x AS5600 (angulo de cada oruga) tras el multiplexor TCA9548A.
 */
#pragma once

#include <stdint.h>
#include <control_core.h>

void encoders_init();

/* Lee el angulo de la oruga 'idx' {FR,FL,RR,RL} en grados (con offset).
 * Lectura de los dos bytes en una unica transaccion I2C (el original leia
 * byte alto y bajo por separado y podia mezclar dos muestras distintas).
 * Devuelve false si el sensor no responde. */
bool encoders_leer(int idx, float *angDeg);
