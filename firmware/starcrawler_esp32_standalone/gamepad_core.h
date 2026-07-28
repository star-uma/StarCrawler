/*
 * gamepad_core.h — StarCrawler STANDALONE (sin PC)
 * ================================================
 * Puerto a C de la lógica de StarCrawlerXbox.py: deadzone, mezcla
 * diferencial, cambio de modo con RB y vectores de los modos 2/3/4.
 *
 * Lógica pura sin dependencias de Arduino/Bluepad32: compilable en PC
 * para los tests unitarios de test/host/test_gamepad_core.cpp.
 *
 * El mando Xbox se conecta por Bluetooth directamente al ESP32 (Bluepad32);
 * este módulo convierte el estado del mando en el mismo cc_DatagramaPC que
 * antes enviaba el PC, así el resto del firmware no cambia.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Estado instantáneo del mando, ya normalizado:
 * ejes en [-1, 1] (adelante/derecha positivos), botones como booleanos. */
typedef struct {
  float ejeIzqY;    /* stick izquierdo vertical: +1 = adelante */
  float ejeDerX;    /* stick derecho horizontal: +1 = derecha */
  bool rb;          /* bumper derecho: cambio de modo */
  bool a, b, x, y;  /* modo 2: 225 / 180 / 135 / 90 grados */
  bool dpadArriba, dpadAbajo, dpadIzq, dpadDer;
  bool trigIzq, trigDer; /* gatillos (ya umbralizados): subir / bajar */
} gc_EstadoMando;

/* Estado persistente del procesador de mando */
typedef struct {
  int16_t modo;                       /* modo activo 1..GC_MODO_MAX */
  bool rbAnterior;                    /* flanco ascendente de RB */
  int16_t ultimoObjetivo[CC_NUM_ORUGAS]; /* última consigna del modo 2 */
} gc_EstadoControl;

/* Número de modos que cicla RB. Sin IMU son 4 (1->2->3->4->1). */
#ifndef GC_MODO_MAX
#define GC_MODO_MAX 4
#endif

void gc_reset(gc_EstadoControl *st);

/* Deadzone con reescalado, idéntica a aplicar_deadzone() del Python. */
float gc_deadzone(float valor, float zona);

/* Procesa una muestra del mando y rellena el datagrama equivalente al del
 * PC (id=1, velocidades dps*100, vector o[], modo). ganancia = GAIN (30),
 * velMaxDps = saturación (40). */
void gc_procesar(gc_EstadoControl *st, const gc_EstadoMando *m,
                 float ganancia, float velMaxDps, cc_DatagramaPC *out);

#ifdef __cplusplus
}
#endif
