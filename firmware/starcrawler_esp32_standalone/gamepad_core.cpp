/*
 * gamepad_core.cpp — StarCrawler STANDALONE (sin PC)
 * ==================================================
 * Puerto fiel de la lógica de control/StarCrawlerXbox.py. Cada función
 * indica la función Python original a la que sustituye.
 */
#include "gamepad_core.h"

#include <math.h>
#include <string.h>

void gc_reset(gc_EstadoControl *st) {
  st->modo = 1; /* arranca en tracción, como el Python */
  st->rbAnterior = false;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) st->ultimoObjetivo[i] = 180;
}

/* aplicar_deadzone(): elimina ruido de reposo y reescala desde el borde */
float gc_deadzone(float valor, float zona) {
  float magnitud = fabsf(valor);
  if (magnitud < zona) return 0.0f;
  float signo = (valor > 0.0f) ? 1.0f : -1.0f;
  return signo * (magnitud - zona) / (1.0f - zona);
}

static int16_t clampInt16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}

/* calcular_traccion(): mezcla diferencial con ganancia y saturación */
static void calcularTraccion(const gc_EstadoMando *m, float ganancia,
                             float velMaxDps, int16_t *izq, int16_t *der) {
  float velBase = m->ejeIzqY * ganancia;
  float diferencial = m->ejeDerX * ganancia;

  float velIzq = cc_saturar(velBase - diferencial, velMaxDps);
  float velDer = cc_saturar(velBase + diferencial, velMaxDps);

  /* El firmware invierte el tren izquierdo internamente, igual que hacía
   * el Arduino con lo que enviaba el PC: aquí se envía sin invertir. */
  *izq = clampInt16(velIzq * 100.0f);
  *der = clampInt16(velDer * 100.0f);
}

/* calcular_modo_2(): posición absoluta con A/B/X/Y (persiste la última) */
static void calcularModo2(gc_EstadoControl *st, const gc_EstadoMando *m,
                          int16_t o[CC_NUM_ORUGAS]) {
  const bool botones[4] = {m->a, m->b, m->x, m->y};
  const int16_t angulos[4] = {225, 180, 135, 90};
  for (int i = 0; i < 4; i++) {
    if (botones[i]) {
      for (int j = 0; j < CC_NUM_ORUGAS; j++) st->ultimoObjetivo[j] = angulos[i];
      break;
    }
  }
  for (int j = 0; j < CC_NUM_ORUGAS; j++) o[j] = st->ultimoObjetivo[j];
}

/* calcular_modo_3(): inclinar las cuatro orugas a la vez con la cruceta */
static void calcularModo3(const gc_EstadoMando *m, int16_t o[CC_NUM_ORUGAS]) {
  if (m->dpadArriba && !m->dpadAbajo && !m->dpadIzq && !m->dpadDer) {
    o[0] = -1; o[1] = 1; o[2] = -1; o[3] = 1;   /* inclinar adelante */
  } else if (m->dpadAbajo && !m->dpadArriba && !m->dpadIzq && !m->dpadDer) {
    o[0] = 1; o[1] = -1; o[2] = 1; o[3] = -1;   /* inclinar atras */
  } else if (m->dpadIzq && !m->dpadArriba && !m->dpadAbajo && !m->dpadDer) {
    o[0] = 1; o[1] = 1; o[2] = -1; o[3] = -1;   /* inclinar izquierda */
  } else if (m->dpadDer && !m->dpadArriba && !m->dpadAbajo && !m->dpadIzq) {
    o[0] = -1; o[1] = -1; o[2] = 1; o[3] = 1;   /* inclinar derecha */
  } else {
    o[0] = 0; o[1] = 0; o[2] = 0; o[3] = 0;
  }
}

/* calcular_modo_4(): cruceta elige el par, gatillos el sentido */
static void calcularModo4(const gc_EstadoMando *m, int16_t o[CC_NUM_ORUGAS]) {
  int16_t sentido = 0;
  if (m->trigDer) sentido = 1;        /* trigger derecho = bajar */
  else if (m->trigIzq) sentido = -1;  /* trigger izquierdo = subir */

  o[0] = 0; o[1] = 0; o[2] = 0; o[3] = 0;
  if (sentido == 0) return;

  if (m->dpadArriba && !m->dpadAbajo && !m->dpadIzq && !m->dpadDer) {
    o[0] = -sentido; o[1] = sentido;              /* par delantero */
  } else if (m->dpadAbajo && !m->dpadArriba && !m->dpadIzq && !m->dpadDer) {
    o[2] = sentido; o[3] = -sentido;              /* par trasero */
  } else if (m->dpadIzq && !m->dpadArriba && !m->dpadAbajo && !m->dpadDer) {
    o[1] = sentido; o[3] = -sentido;              /* par izquierdo */
  } else if (m->dpadDer && !m->dpadArriba && !m->dpadAbajo && !m->dpadIzq) {
    o[0] = -sentido; o[2] = sentido;              /* par derecho */
  }
}

/* Bucle principal del Python: flanco de RB + construir_datagrama() */
void gc_procesar(gc_EstadoControl *st, const gc_EstadoMando *m,
                 float ganancia, float velMaxDps, cc_DatagramaPC *out) {
  /* Cambio de modo con flanco ascendente de RB: 1->2->...->GC_MODO_MAX->1 */
  if (m->rb && !st->rbAnterior) {
    st->modo = (int16_t)((st->modo % GC_MODO_MAX) + 1);
  }
  st->rbAnterior = m->rb;

  memset(out, 0, sizeof(*out));
  out->id = 1; /* mismo origen que el PC: el resto del firmware no cambia */
  out->mode = st->modo;

  switch (st->modo) {
    case 1:
      calcularTraccion(m, ganancia, velMaxDps,
                       &out->action_left_train, &out->action_right_train);
      break;
    case 2:
      calcularModo2(st, m, out->o);
      break;
    case 3:
      calcularModo3(m, out->o);
      break;
    case 4:
      calcularModo4(m, out->o);
      break;
    default:
      break;
  }
}
