/*
 * steppers.h
 * ==========
 * Control de los 4 motores paso a paso de elevación (drivers DM542).
 * Un único timer hardware genera el tren de pulsos STEP en la ISR,
 * igual que el firmware ESP32 original, pero con escritura directa
 * de registros GPIO (segura en IRAM y más rápida que digitalWrite).
 *
 * Orden de motores en todos los vectores: {FR, FL, RR, RL}.
 */
#pragma once

#include <stdint.h>

void steppers_init();

/* cmd: +1 = aumentar ángulo (sentido "horario" de las tablas del TFG),
 *      -1 = disminuir, 0 = parar. */
void steppers_comando(int motor, int8_t cmd);

void steppers_pararTodos();

/* true si algún motor está generando pasos */
bool steppers_algunoActivo();
