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
  bool rb;          /* R1: cambio de modo (TFG) / par trasero sube (simultáneo) */
  bool a, b, x, y;  /* posiciones absolutas 225 / 180 / 135 / 90 grados */
  bool dpadArriba, dpadAbajo, dpadIzq, dpadDer;
  bool trigIzq, trigDer; /* gatillos (ya umbralizados): subir / bajar */
  /* Botones adicionales del esquema simultáneo */
  bool l1;          /* par delantero sube */
  bool l2;          /* par delantero baja */
  bool r2;          /* par trasero baja */
  bool l3;          /* clic stick izq: alternar velocidad lenta/rápida */
  bool share;       /* parada de emergencia (mantener) */
  bool options;     /* nivelado automático (mantener; requiere IMU) */
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

/* ═══ Esquema SIMULTÁNEO (sin modos) ══════════════════════════════════════
 * Tracción siempre activa + orugas superpuestas:
 *   L1/L2 = par delantero sube/baja   R1/R2 = par trasero sube/baja
 *   Cruceta = inclinar el conjunto    ○△□✕ = presets de pose (mantener)
 *   SHARE = parada de emergencia      L3 = velocidad lenta/rápida
 *   OPTIONS = nivelado (con IMU)
 * "Subir" = el brazo se levanta del suelo. Pose vertical arriba del TFG
 * {FR,FL,RR,RL} = {90,270,270,90}, de donde sale la tabla de sentidos y el
 * espejado de los presets (FR/RL = a, FL/RR = 360-a).
 */

#ifndef GC_MS_PRESET
#define GC_MS_PRESET 500u        /* mantener un preset para activarlo (ms) */
#endif
#ifndef GC_FACTOR_VEL_LENTA
#define GC_FACTOR_VEL_LENTA 0.5f /* ganancia en velocidad lenta */
#endif

typedef struct {
  bool velocidadLenta;
  bool l3Anterior;
  int8_t presetPulsado;           /* -1 o 0..3 (a,b,x,y) mientras se mantiene */
  uint32_t msInicioPreset;
  bool posicionVigente;           /* hay un preset de posición en curso */
  int16_t objetivo[CC_NUM_ORUGAS];
} gc_EstadoSimultaneo;

typedef struct {
  int16_t action_left_train;      /* dps*100, ya con ganancia lenta/rápida */
  int16_t action_right_train;
  int8_t oruga[CC_NUM_ORUGAS];    /* comando incremental -1/0/+1 {FR,FL,RR,RL} */
  bool usarPosicion;              /* true: ignorar oruga[] y usar objetivo[] */
  int16_t objetivo[CC_NUM_ORUGAS];
  bool paradaEmergencia;          /* SHARE mantenido */
  bool nivelar;                   /* OPTIONS mantenido */
  bool velocidadLenta;            /* estado del toggle L3 */
} gc_SalidaSimultanea;

void gc_simultaneoReset(gc_EstadoSimultaneo *st);

/* ahoraMs = millis() (para el temporizador de presets; robusto ante wrap). */
void gc_procesarSimultaneo(gc_EstadoSimultaneo *st, const gc_EstadoMando *m,
                           uint32_t ahoraMs, float ganancia, float velMaxDps,
                           gc_SalidaSimultanea *out);

#ifdef __cplusplus
}
#endif
