/*
 * control_core.h — StarCrawler BÁSICO (sin IMU)
 * =============================================
 * Lógica pura de control, subconjunto sin nivelado/IMU de la versión
 * canónica y testeada en firmware/starcrawler_esp32/control_core.h
 * (los tests de test/host/ cubren estas mismas funciones).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_NUM_ORUGAS 4

/* Datagrama PC -> robot: 9 x int16 little-endian (18 bytes) */
typedef struct {
  int16_t id;                 /* 1 = PC */
  int16_t action_left_train;  /* dps * 100 */
  int16_t action_right_train; /* dps * 100 */
  int16_t o[CC_NUM_ORUGAS];   /* consignas orugas {FR, FL, RR, RL} */
  int16_t mode;               /* 1..4 (5 se ignora en esta versión) */
  int16_t code_error;
} cc_DatagramaPC;

/* Bits de error de la telemetría */
#define CC_ERR_ENCODER_FR (1u << 0)
#define CC_ERR_ENCODER_FL (1u << 1)
#define CC_ERR_ENCODER_RR (1u << 2)
#define CC_ERR_ENCODER_RL (1u << 3)
#define CC_ERR_CAN        (1u << 5)
#define CC_ERR_WATCHDOG   (1u << 6)

bool cc_parseDatagrama(const uint8_t *buf, int len, cc_DatagramaPC *out);

/* Telemetría (18 bytes): [0]=2, [1..4]=ángulos*100, [5..6]=reservado (0),
 * [7]=modo activo, [8]=bits de error. */
int cc_construirTelemetria(uint8_t *buf, const float ang[CC_NUM_ORUGAS],
                           int16_t modo, uint16_t errores);

float cc_saturar(float valor, float maximo);
float cc_rateLimiter(float actual, float consigna, float maxDelta);

void cc_tramaVelocidadRMD(float velocidad_dps, uint8_t out[8]);
void cc_tramaLiberarRMD(uint8_t out[8]);

float cc_as5600ADeg(uint16_t raw, float offsetDeg);

int8_t cc_controlPosicion(float actual, float objetivo, bool enMarcha,
                          float umbralArranque, float umbralParada);

int8_t cc_aplicarLimites(int8_t cmd, float angulo, bool encoderOk,
                         float minDeg, float maxDeg);

bool cc_watchdogExpirado(uint32_t ahoraMs, uint32_t ultimoPaqueteMs,
                         uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif
