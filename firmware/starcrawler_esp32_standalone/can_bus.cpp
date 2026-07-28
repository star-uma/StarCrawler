/*
 * can_bus.cpp
 * ===========
 * Implementación de los dos backends CAN. Ver can_bus.h.
 */
#include "config.h"
#include "can_bus.h"

#if CAN_BACKEND == CAN_BACKEND_MCP2515
/* ═══ Backend MCP2515 (ACAN2515, modo polling) ════════════════════════════
 * Polling en vez de interrupción: evita transacciones SPI dentro de una ISR
 * y a 100 Hz de lazo sobra para el tráfico de este robot (200 tramas/s TX).
 */
#include <SPI.h>
#include <ACAN2515.h>

static ACAN2515 mcp(PIN_CAN_CS, SPI, 255); /* 255 = sin pin de interrupción */

bool canbus_init() {
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);
  /* Cristal de 16 MHz obligatorio para 1 Mbps (ver docs/) */
  ACAN2515Settings ajustes(16UL * 1000UL * 1000UL, 1000UL * 1000UL);
  ajustes.mRequestedMode = ACAN2515Settings::NormalMode;
  const uint16_t error = mcp.begin(ajustes, NULL); /* NULL = modo polling */
  return error == 0;
}

bool canbus_enviar(uint32_t id, const uint8_t datos[8]) {
  CANMessage msg;
  msg.id = id;
  msg.ext = false;
  msg.rtr = false;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.data[i] = datos[i];
  return mcp.tryToSend(msg);
}

bool canbus_recibir(uint32_t *id, uint8_t datos[8]) {
  CANMessage msg;
  if (!mcp.receive(msg)) return false;
  *id = msg.id;
  for (int i = 0; i < 8; i++) datos[i] = msg.data[i];
  return true;
}

void canbus_atender() {
  mcp.poll();
}

#elif CAN_BACKEND == CAN_BACKEND_TWAI
/* ═══ Backend TWAI (controlador CAN interno del ESP32) ════════════════════ */
#include "driver/twai.h"

bool canbus_init() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_TWAI_TX, (gpio_num_t)PIN_TWAI_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 8;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

bool canbus_enviar(uint32_t id, const uint8_t datos[8]) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = 8;
  for (int i = 0; i < 8; i++) msg.data[i] = datos[i];
  return twai_transmit(&msg, 0) == ESP_OK;
}

bool canbus_recibir(uint32_t *id, uint8_t datos[8]) {
  twai_message_t msg;
  if (twai_receive(&msg, 0) != ESP_OK) return false;
  *id = msg.identifier;
  for (int i = 0; i < 8; i++) datos[i] = msg.data[i];
  return true;
}

void canbus_atender() { /* nada que hacer: el TWAI tiene colas hardware */ }

#else
#error "CAN_BACKEND no válido: usa CAN_BACKEND_MCP2515 o CAN_BACKEND_TWAI"
#endif
