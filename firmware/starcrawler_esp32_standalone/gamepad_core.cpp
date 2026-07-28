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

/* ═══ Esquema SIMULTÁNEO (sin modos) ══════════════════════════════════════ */

/* Comandos incrementales para que el brazo SUBA (se levante del suelo),
 * derivados de la pose vertical arriba {90,270,270,90} del TFG:
 * FR baja de angulo (-1), FL sube (+1), RR sube (+1), RL baja (-1). */
static const int8_t TABLA_SUBIR[CC_NUM_ORUGAS] = {-1, +1, +1, -1};

/* Espejado por oruga de un angulo de preset: FR/RL = a, FL/RR = 360-a. */
static int16_t espejoPreset(int16_t a, int i) {
  return (i == 0 || i == 3) ? a : (int16_t)(360 - a);
}

void gc_simultaneoReset(gc_EstadoSimultaneo *st) {
  st->velocidadLenta = false;
  st->l3Anterior = false;
  st->presetPulsado = -1;
  st->msInicioPreset = 0;
  st->posicionVigente = false;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) st->objetivo[i] = 180;
}

void gc_procesarSimultaneo(gc_EstadoSimultaneo *st, const gc_EstadoMando *m,
                           uint32_t ahoraMs, float ganancia, float velMaxDps,
                           gc_SalidaSimultanea *out) {
  memset(out, 0, sizeof(*out));

  /* Toggle de velocidad lenta con flanco ascendente de L3 */
  if (m->l3 && !st->l3Anterior) st->velocidadLenta = !st->velocidadLenta;
  st->l3Anterior = m->l3;
  out->velocidadLenta = st->velocidadLenta;

  /* Parada de emergencia: SHARE manda sobre todo y cancela presets */
  if (m->share) {
    st->posicionVigente = false;
    st->presetPulsado = -1;
    out->paradaEmergencia = true;
    return;
  }

  /* Traccion SIEMPRE activa (misma mezcla diferencial del TFG) */
  const float g = st->velocidadLenta ? ganancia * GC_FACTOR_VEL_LENTA : ganancia;
  {
    float velBase = m->ejeIzqY * g;
    float diferencial = m->ejeDerX * g;
    out->action_left_train =
        clampInt16(cc_saturar(velBase - diferencial, velMaxDps) * 100.0f);
    out->action_right_train =
        clampInt16(cc_saturar(velBase + diferencial, velMaxDps) * 100.0f);
  }

  /* Orugas manual: pares (L1/L2 delante, R1/R2 detras) + cruceta, sumados */
  int manual[CC_NUM_ORUGAS] = {0, 0, 0, 0};
  int sentidoDelante = (m->l1 ? 1 : 0) - (m->l2 ? 1 : 0); /* ambos = 0 */
  int sentidoDetras  = (m->rb ? 1 : 0) - (m->r2 ? 1 : 0);
  manual[0] += sentidoDelante * TABLA_SUBIR[0]; /* FR */
  manual[1] += sentidoDelante * TABLA_SUBIR[1]; /* FL */
  manual[2] += sentidoDetras * TABLA_SUBIR[2];  /* RR */
  manual[3] += sentidoDetras * TABLA_SUBIR[3];  /* RL */

  int16_t inclinacion[CC_NUM_ORUGAS];
  calcularModo3(m, inclinacion); /* vectores de la cruceta, como el modo 3 */
  bool hayManual = (sentidoDelante != 0) || (sentidoDetras != 0);
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    manual[i] += inclinacion[i];
    if (inclinacion[i] != 0) hayManual = true;
    out->oruga[i] = (int8_t)((manual[i] > 0) - (manual[i] < 0));
  }

  /* Cualquier entrada manual cancela el preset en curso */
  if (hayManual) st->posicionVigente = false;

  /* Presets de pose: mantener un boton GC_MS_PRESET ms para activarlo */
  const bool botones[4] = {m->a, m->b, m->x, m->y};
  const int16_t angulos[4] = {225, 180, 135, 90};
  int8_t pulsado = -1;
  for (int i = 0; i < 4; i++) {
    if (botones[i]) { pulsado = (int8_t)i; break; }
  }
  if (pulsado < 0) {
    st->presetPulsado = -1;
  } else {
    if (st->presetPulsado != pulsado) {
      st->presetPulsado = pulsado;
      st->msInicioPreset = ahoraMs;
    } else if ((uint32_t)(ahoraMs - st->msInicioPreset) >= GC_MS_PRESET) {
      st->posicionVigente = true;
      for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        st->objetivo[i] = espejoPreset(angulos[pulsado], i);
      }
    }
  }

  if (st->posicionVigente && !hayManual) {
    out->usarPosicion = true;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) out->objetivo[i] = st->objetivo[i];
  }

  out->nivelar = m->options;
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
