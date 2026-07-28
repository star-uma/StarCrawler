/*
 * encoders.cpp — StarCrawler BÁSICO (sin IMU)
 * ===========================================
 */
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "encoders.h"

static const float offsetsEncoder[CC_NUM_ORUGAS] = OFFSETS_ENCODER;

static bool tcaSeleccionar(uint8_t canal) {
  if (canal > 7) return false;
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

void encoders_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  /* Timeout corto: con sensores desconectados (pruebas de mesa) una
   * transaccion fallida no debe bloquear el lazo de control. */
  Wire.setTimeOut(5);
}

bool encoders_leer(int idx, float *angDeg) {
  if (idx < 0 || idx >= CC_NUM_ORUGAS) return false;
  if (!tcaSeleccionar(idx)) return false;

  /* RAW ANGLE: alto 0x0C + bajo 0x0D en una sola transacción */
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
