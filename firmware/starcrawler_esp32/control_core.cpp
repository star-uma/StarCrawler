/*
 * control_core.cpp
 * ================
 * Implementación de la lógica pura de control. Ver control_core.h.
 * Compilable tanto en el ESP32 como en PC (tests de test/host/).
 */
#include "control_core.h"

#include <math.h>
#include <string.h>

/* ── Utilidades internas ────────────────────────────────────────────────── */

static int16_t leerInt16LE(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void escribirInt16LE(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

static int8_t saturarInt8(int v) {
  if (v > 1) return 1;
  if (v < -1) return -1;
  return (int8_t)v;
}

/* ── Comunicaciones ─────────────────────────────────────────────────────── */

bool cc_parseDatagrama(const uint8_t *buf, int len, cc_DatagramaPC *out) {
  if (len < 18 || buf == NULL || out == NULL) return false;
  out->id                 = leerInt16LE(buf + 0);
  out->action_left_train  = leerInt16LE(buf + 2);
  out->action_right_train = leerInt16LE(buf + 4);
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    out->o[i] = leerInt16LE(buf + 6 + 2 * i);
  }
  out->mode       = leerInt16LE(buf + 14);
  out->code_error = leerInt16LE(buf + 16);
  return true;
}

int cc_construirTelemetria(uint8_t *buf, const float ang[CC_NUM_ORUGAS],
                           float roll, float pitch, int16_t modo,
                           uint16_t errores) {
  escribirInt16LE(buf + 0, 2); /* origen: robot */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    escribirInt16LE(buf + 2 + 2 * i, (int16_t)lrintf(ang[i] * 100.0f));
  }
  escribirInt16LE(buf + 10, (int16_t)lrintf(roll * 100.0f));
  escribirInt16LE(buf + 12, (int16_t)lrintf(pitch * 100.0f));
  escribirInt16LE(buf + 14, modo);
  escribirInt16LE(buf + 16, (int16_t)errores);
  return 18;
}

/* ── Tracción ───────────────────────────────────────────────────────────── */

float cc_saturar(float valor, float maximo) {
  if (valor > maximo) return maximo;
  if (valor < -maximo) return -maximo;
  return valor;
}

float cc_rateLimiter(float actual, float consigna, float maxDelta) {
  float delta = consigna - actual;
  if (delta > maxDelta) return actual + maxDelta;
  if (delta < -maxDelta) return actual - maxDelta;
  return consigna;
}

void cc_tramaVelocidadRMD(float velocidad_dps, uint8_t out[8]) {
  /* Protocolo RMD V4.2: comando 0xA2, velocidad en unidades de 0.01 dps */
  int32_t speedControl = (int32_t)lrintf(velocidad_dps * 100.0f);
  out[0] = 0xA2;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = 0x00;
  out[4] = (uint8_t)(speedControl & 0xFF);
  out[5] = (uint8_t)((speedControl >> 8) & 0xFF);
  out[6] = (uint8_t)((speedControl >> 16) & 0xFF);
  out[7] = (uint8_t)((speedControl >> 24) & 0xFF);
}

void cc_tramaLiberarRMD(uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = 0x80;
}

/* ── Elevación ──────────────────────────────────────────────────────────── */

float cc_as5600ADeg(uint16_t raw, float offsetDeg) {
  /* 12 bits: 360 / 4096 = 0.087890625 grados por cuenta */
  return (float)raw * 0.087890625f + offsetDeg;
}

int8_t cc_controlPosicion(float actual, float objetivo, bool enMarcha,
                          float umbralArranque, float umbralParada) {
  float error = objetivo - actual;
  float magnitud = fabsf(error);

  if (enMarcha) {
    if (magnitud <= umbralParada) return 0;
  } else {
    if (magnitud <= umbralArranque) return 0;
  }
  return (error > 0.0f) ? 1 : -1;
}

void cc_nivelado(float roll, float pitch, float limiteDeg,
                 int8_t Y[CC_NUM_ORUGAS]) {
  /* Puerto literal del Chart "Process leveling vector" (Código 7.4 del TFG).
   * Orden {FR, FL, RR, RL}, sentidos respecto de F.R. como en modos 3 y 4. */
  int yp[CC_NUM_ORUGAS] = {0, 0, 0, 0};
  int yr[CC_NUM_ORUGAS] = {0, 0, 0, 0};

  if (pitch > limiteDeg) {
    yp[0] = 1; yp[1] = -1; yp[2] = 1; yp[3] = -1;
  } else if (pitch < -limiteDeg) {
    yp[0] = -1; yp[1] = 1; yp[2] = -1; yp[3] = 1;
  }

  if (roll > limiteDeg) {
    yr[0] = 1; yr[1] = 1; yr[2] = -1; yr[3] = -1;
  } else if (roll < -limiteDeg) {
    yr[0] = -1; yr[1] = -1; yr[2] = 1; yr[3] = 1;
  }

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    Y[i] = saturarInt8(yp[i] + yr[i]);
  }
}

int8_t cc_aplicarLimites(int8_t cmd, float angulo, bool encoderOk,
                         float minDeg, float maxDeg) {
  if (!encoderOk) return cmd;
  if (cmd > 0 && angulo >= maxDeg) return 0;
  if (cmd < 0 && angulo <= minDeg) return 0;
  return cmd;
}

/* ── Seguridad ──────────────────────────────────────────────────────────── */

bool cc_watchdogExpirado(uint32_t ahoraMs, uint32_t ultimoPaqueteMs,
                         uint32_t timeoutMs) {
  return (uint32_t)(ahoraMs - ultimoPaqueteMs) > timeoutMs;
}

/* ── Filtro complementario ──────────────────────────────────────────────── */

void cc_filtroActitudReset(cc_FiltroActitud *f) {
  f->roll = 0.0f;
  f->pitch = 0.0f;
  f->inicializado = false;
}

void cc_filtroActitudUpdate(cc_FiltroActitud *f,
                            float rollAccelDeg, float pitchAccelDeg,
                            float rollRateDps, float pitchRateDps,
                            float dt, float alpha) {
  if (!f->inicializado) {
    /* Primera muestra: sembrar directamente con el acelerómetro */
    f->roll = rollAccelDeg;
    f->pitch = pitchAccelDeg;
    f->inicializado = true;
    return;
  }
  f->roll  = alpha * (f->roll  + rollRateDps  * dt) + (1.0f - alpha) * rollAccelDeg;
  f->pitch = alpha * (f->pitch + pitchRateDps * dt) + (1.0f - alpha) * pitchAccelDeg;
}
