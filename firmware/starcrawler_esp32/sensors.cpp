/*
 * sensors.cpp
 * ===========
 * Implementación de encoders AS5600 (vía TCA9548A) y MPU9250.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "sensors.h"

static const float offsetsEncoder[CC_NUM_ORUGAS] = OFFSETS_ENCODER;

/* ── TCA9548A ───────────────────────────────────────────────────────────── */

static bool tcaSeleccionar(uint8_t canal) {
  if (canal > 7) return false;
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

/* ── Encoders AS5600 ────────────────────────────────────────────────────── */

void encoders_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); /* fast mode: 4 encoders + IMU en <2 ms por ciclo */
}

bool encoders_leer(int idx, float *angDeg) {
  if (idx < 0 || idx >= CC_NUM_ORUGAS) return false;
  if (!tcaSeleccionar(idx)) return false;

  /* RAW ANGLE: registro alto 0x0C, bajo 0x0D. Lectura secuencial de ambos
   * en una única transacción para que la muestra sea coherente. */
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)DIR_AS5600, 2) != 2) return false;
  uint16_t alto = Wire.read();
  uint16_t bajo = Wire.read();
  uint16_t raw = (uint16_t)(((alto & 0x0F) << 8) | bajo);

  *angDeg = cc_as5600ADeg(raw, offsetsEncoder[idx]);
  return true;
}

/* ── MPU9250 ────────────────────────────────────────────────────────────── */

static float gyroBias[3] = {0.0f, 0.0f, 0.0f};
static bool imuPresente = false;

static bool mpuEscribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(DIR_MPU9250);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

/* Lee acelerómetro y giroscopio crudos (14 bytes desde 0x3B). */
static bool mpuLeerCrudo(int16_t acc[3], int16_t gyr[3]) {
  Wire.beginTransmission(DIR_MPU9250);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_MPU9250, 14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  acc[0] = (int16_t)((b[0] << 8) | b[1]);
  acc[1] = (int16_t)((b[2] << 8) | b[3]);
  acc[2] = (int16_t)((b[4] << 8) | b[5]);
  /* b[6..7] = temperatura, no usada */
  gyr[0] = (int16_t)((b[8] << 8) | b[9]);
  gyr[1] = (int16_t)((b[10] << 8) | b[11]);
  gyr[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

bool imu_init() {
  /* ¿Responde en el bus? */
  Wire.beginTransmission(DIR_MPU9250);
  if (Wire.endTransmission() != 0) {
    imuPresente = false;
    return false;
  }

  bool ok = true;
  ok &= mpuEscribir(0x6B, 0x00); /* PWR_MGMT_1: salir de sleep */
  delay(50);
  ok &= mpuEscribir(0x1B, 0x00); /* GYRO_CONFIG: ±250 dps (131 LSB/dps) */
  ok &= mpuEscribir(0x1C, 0x00); /* ACCEL_CONFIG: ±2 g (16384 LSB/g) */
  ok &= mpuEscribir(0x1A, 0x03); /* CONFIG: DLPF 41 Hz */

  imuPresente = ok;
  return ok;
}

void imu_calibrarGyro() {
  if (!imuPresente) return;

  float suma[3] = {0.0f, 0.0f, 0.0f};
  int validas = 0;
  int16_t acc[3], gyr[3];

  for (int i = 0; i < MUESTRAS_CAL_GYRO; i++) {
    if (mpuLeerCrudo(acc, gyr)) {
      for (int e = 0; e < 3; e++) suma[e] += (float)gyr[e];
      validas++;
    }
    delay(2);
  }

  if (validas > 0) {
    for (int e = 0; e < 3; e++) gyroBias[e] = suma[e] / (float)validas;
  }
}

bool imu_actualizar(cc_FiltroActitud *filtro, float dt) {
  if (!imuPresente) return false;

  int16_t acc[3], gyr[3];
  if (!mpuLeerCrudo(acc, gyr)) return false;

  const float ax = (float)acc[0] / 16384.0f;
  const float ay = (float)acc[1] / 16384.0f;
  const float az = (float)acc[2] / 16384.0f;
  const float gx = ((float)gyr[0] - gyroBias[0]) / 131.0f;
  const float gy = ((float)gyr[1] - gyroBias[1]) / 131.0f;
  const float gz = ((float)gyr[2] - gyroBias[2]) / 131.0f;

  /* Criterio del TFG (fig. 7.40): IMU anclada al frontal del chasis con
   * eje Z longitudinal, Y transversal, X vertical:
   *   pitch (giro sobre Y): + inclinado hacia atrás  -> accel: atan2(az, ax)
   *   roll  (giro sobre Z): + inclinado a la derecha -> accel: atan2(ay, ax)
   * ⚠ Verificar signos y ejes con el robot en el suelo antes del modo 5. */
  const float RAD_A_DEG = 57.29577951f;
  const float pitchAccel = SIGNO_PITCH * atan2f(az, ax) * RAD_A_DEG;
  const float rollAccel  = SIGNO_ROLL  * atan2f(ay, ax) * RAD_A_DEG;
  const float pitchRate  = SIGNO_PITCH * gy;
  const float rollRate   = SIGNO_ROLL  * gz;

  cc_filtroActitudUpdate(filtro, rollAccel, pitchAccel, rollRate, pitchRate,
                         dt, ALPHA_FILTRO);
  return true;
}
