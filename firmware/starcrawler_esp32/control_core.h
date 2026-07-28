/*
 * control_core.h
 * ==============
 * Lógica pura de control de StarCrawler (sin dependencias de Arduino).
 *
 * Este módulo contiene toda la lógica de decisión del robot: parseo de
 * datagramas, saturación, rate limiter, tramas CAN RMD, control de posición
 * con histéresis, nivelado automático (Código 7.4 del TFG) y watchdog.
 *
 * Al no depender de Arduino, se compila también en el PC con g++ para los
 * tests unitarios de test/host/.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_NUM_ORUGAS 4

/* Datagrama PC -> robot: 9 x int16 little-endian (18 bytes).
 * Mismo formato que StarCrawlerXbox.py y que el firmware MKR original. */
typedef struct {
  int16_t id;                 /* 1 = PC */
  int16_t action_left_train;  /* dps * 100 */
  int16_t action_right_train; /* dps * 100 */
  int16_t o[CC_NUM_ORUGAS];   /* consignas orugas {FR, FL, RR, RL} */
  int16_t mode;               /* 1..5 */
  int16_t code_error;
} cc_DatagramaPC;

/* Bits de error de la telemetría robot -> PC */
#define CC_ERR_ENCODER_FR (1u << 0)
#define CC_ERR_ENCODER_FL (1u << 1)
#define CC_ERR_ENCODER_RR (1u << 2)
#define CC_ERR_ENCODER_RL (1u << 3)
#define CC_ERR_IMU        (1u << 4)
#define CC_ERR_CAN        (1u << 5)
#define CC_ERR_WATCHDOG   (1u << 6)

/* ── Comunicaciones ─────────────────────────────────────────────────────── */

/* Parsea un datagrama de 18 bytes little-endian. Devuelve false si len < 18. */
bool cc_parseDatagrama(const uint8_t *buf, int len, cc_DatagramaPC *out);

/* Construye la telemetría robot -> PC (18 bytes, 9 x int16 LE):
 * [0]=2 (origen robot), [1..4]=ángulos*100 {FR,FL,RR,RL},
 * [5]=roll*100, [6]=pitch*100, [7]=modo activo, [8]=bits de error.
 * Devuelve el número de bytes escritos (18). */
int cc_construirTelemetria(uint8_t *buf, const float ang[CC_NUM_ORUGAS],
                           float roll, float pitch, int16_t modo,
                           uint16_t errores);

/* ── Tracción (motores RMD-X8 por CAN) ──────────────────────────────────── */

float cc_saturar(float valor, float maximo);

/* Limita la variación de 'actual' hacia 'consigna' a maxDelta por llamada. */
float cc_rateLimiter(float actual, float consigna, float maxDelta);

/* Trama de velocidad RMD (0xA2): [0]=0xA2 [1..3]=0 [4..7]=dps*100 int32 LE */
void cc_tramaVelocidadRMD(float velocidad_dps, uint8_t out[8]);

/* Trama de liberación RMD (0x80): motor queda sin par */
void cc_tramaLiberarRMD(uint8_t out[8]);

/* ── Elevación (encoders + steppers) ────────────────────────────────────── */

/* Convierte lectura cruda AS5600 (0..4095) a grados con offset de montaje. */
float cc_as5600ADeg(uint16_t raw, float offsetDeg);

/* Control de posición con histéresis. Devuelve el sentido de giro:
 *   +1 = aumentar ángulo, -1 = disminuir, 0 = parado.
 * Si el motor está parado solo arranca con |error| > umbralArranque;
 * si está en marcha solo para con |error| <= umbralParada.
 * (El original usaba un único umbral de 1 grado; la histéresis evita
 *  oscilación alrededor de la consigna.) */
int8_t cc_controlPosicion(float actual, float objetivo, bool enMarcha,
                          float umbralArranque, float umbralParada);

/* Nivelado automático — puerto exacto del Chart de Simulink del TFG
 * (Código 7.4, apartado 7.5.5). Orden del vector: {FR, FL, RR, RL}. */
void cc_nivelado(float roll, float pitch, float limiteDeg,
                 int8_t Y[CC_NUM_ORUGAS]);

/* Límites software de recorrido. Si el encoder es válido y el comando
 * empuja el ángulo fuera de [minDeg, maxDeg], lo anula. Con encoder
 * inválido devuelve el comando sin tocar (decide el llamador). */
int8_t cc_aplicarLimites(int8_t cmd, float angulo, bool encoderOk,
                         float minDeg, float maxDeg);

/* ── Seguridad ──────────────────────────────────────────────────────────── */

/* true si ha pasado más de timeoutMs desde el último paquete.
 * Aritmética sin signo: robusta frente al desbordamiento de millis(). */
bool cc_watchdogExpirado(uint32_t ahoraMs, uint32_t ultimoPaqueteMs,
                         uint32_t timeoutMs);

/* ── Filtro complementario de actitud (roll/pitch) ──────────────────────── */

typedef struct {
  float roll;   /* grados */
  float pitch;  /* grados */
  bool inicializado;
} cc_FiltroActitud;

void cc_filtroActitudReset(cc_FiltroActitud *f);

/* rollAccel/pitchAccel: ángulos calculados del acelerómetro (grados).
 * rollRate/pitchRate: velocidades angulares del giroscopio (dps).
 * dt en segundos; alpha típico 0.98 (peso del giroscopio). */
void cc_filtroActitudUpdate(cc_FiltroActitud *f,
                            float rollAccelDeg, float pitchAccelDeg,
                            float rollRateDps, float pitchRateDps,
                            float dt, float alpha);

#ifdef __cplusplus
}
#endif
