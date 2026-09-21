/*
 * control_core.cpp — StarCrawler BÁSICO (sin IMU)
 * ===============================================
 * Subconjunto de firmware/starcrawler_esp32/control_core.cpp (versión
 * canónica testeada). Idéntico salvo: sin nivelado, sin filtro de actitud,
 * y la telemetría rellena roll/pitch a cero.
 */
#include "control_core.h"

#include <math.h>
#include <string.h>

static int16_t leerInt16LE(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void escribirInt16LE(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

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
                           int16_t modo, uint16_t errores) {
  escribirInt16LE(buf + 0, 2); /* origen: robot */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    escribirInt16LE(buf + 2 + 2 * i, (int16_t)lrintf(ang[i] * 100.0f));
  }
  escribirInt16LE(buf + 10, 0); /* reservado (roll en la versión completa) */
  escribirInt16LE(buf + 12, 0); /* reservado (pitch en la versión completa) */
  escribirInt16LE(buf + 14, modo);
  escribirInt16LE(buf + 16, (int16_t)errores);
  return 18;
}

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

float cc_as5600ADeg(uint16_t raw, float offsetDeg) {
  return (float)raw * 0.087890625f + offsetDeg; /* 360/4096 */
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

int8_t cc_aplicarLimites(int8_t cmd, float angulo, bool encoderOk,
                         float minDeg, float maxDeg) {
  if (!encoderOk) return cmd;
  if (cmd > 0 && angulo >= maxDeg) return 0;
  if (cmd < 0 && angulo <= minDeg) return 0;
  return cmd;
}

bool cc_watchdogExpirado(uint32_t ahoraMs, uint32_t ultimoPaqueteMs,
                         uint32_t timeoutMs) {
  return (uint32_t)(ahoraMs - ultimoPaqueteMs) > timeoutMs;
}
