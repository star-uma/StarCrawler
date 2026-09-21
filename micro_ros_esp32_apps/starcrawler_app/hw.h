/*
 * hw.h - StarCrawler - capa de hardware sobre ESP-IDF
 * ====================================================================
 * Mismas firmas que los modulos de la version Arduino (can_bus.h,
 * steppers.h, encoders.h) pero implementadas con drivers de ESP-IDF.
 *
 * El objetivo de mantener las firmas es que control_core.c siga valiendo
 * sin tocar una linea: es C puro y esta cubierto por los tests de host.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* --- Bus CAN (TWAI interno + transceptor externo) ------------------- */

bool canbus_init(void);
bool canbus_enviar(uint32_t id, const uint8_t datos[8]);
bool canbus_recibir(uint32_t *id, uint8_t datos[8]);

/* --- Steppers de elevacion (DM542) ---------------------------------- */

void steppers_init(void);

/* cmd: +1 aumenta el angulo de encoder, -1 lo disminuye, 0 para. */
void steppers_comando(int motor, int8_t cmd);
void steppers_pararTodos(void);
bool steppers_algunoActivo(void);

/* --- Encoders (AS5600 tras multiplexor TCA9548A) -------------------- */

void encoders_init(void);
bool encoders_leer(int idx, float *angDeg);

/* --- Utilidades ------------------------------------------------------ */

uint32_t hw_millis(void);
