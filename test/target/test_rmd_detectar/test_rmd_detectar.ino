/*
 * ================================================================
 *  test_rmd_detectar -- que motor RMD hay en el bus (NO MUEVE NADA)
 * ================================================================
 *
 *  Prueba IDs 1..32 (0x141..0x160) a 1 Mbps y a 500 kbps. A cada motor
 *  que contesta le pide modelo (0xB5), version (0xB2), estado 1 (0x9A),
 *  estado 2 (0x9C) y ganancias PID (0x30), y escribe las tramas en bruto
 *  ademas de lo que se entiende de ellas: el formato cambia entre las
 *  versiones del protocolo (V3 / V4).
 *
 *  ESP32 DevKit V1 + transceptor: TX GPIO5, RX GPIO35 (como test_can_esp32).
 *  Comandos: d = detectar, h = ayuda.  115200 baudios.
 * ================================================================
 */

#include "driver/twai.h"

#define PIN_TWAI_TX GPIO_NUM_5
#define PIN_TWAI_RX GPIO_NUM_35
#define ID_MAX 32

static bool arrancar(bool unMega) {
  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 4;
  g.rx_queue_len = 32;
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t1m = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, unMega ? &t1m : &t500, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

static void parar() {
  twai_stop();
  twai_driver_uninstall();
}

static bool busVivo() {
  twai_status_info_t s;
  return twai_get_status_info(&s) == ESP_OK && s.state == TWAI_STATE_RUNNING;
}

static void imprimirEstadoBus() {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;
  Serial.printf("    bus: estado %d, err TX %lu, err RX %lu, fallos TX %lu\n",
                (int)s.state, (unsigned long)s.tx_error_counter,
                (unsigned long)s.rx_error_counter, (unsigned long)s.tx_failed_count);
}

/* Pregunta cmd al motor id; devuelve los bytes de la respuesta (0 = nada).
 * Acepta respuesta en 0x140+id (V3) o 0x240+id (V4). */
static int preguntar(uint8_t id, uint8_t cmd, uint8_t r[8], uint32_t *idResp) {
  if (!busVivo()) return 0;
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {}
  twai_message_t m = {};
  m.identifier = 0x140 + id;
  m.data_length_code = 8;
  m.data[0] = cmd;
  if (twai_transmit(&m, pdMS_TO_TICKS(10)) != ESP_OK) return 0;
  uint32_t fin = millis() + 25;
  while (millis() < fin) {
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK &&
        (rx.identifier == 0x140u + id || rx.identifier == 0x240u + id) &&
        rx.data_length_code > 0 && rx.data[0] == cmd) {
      int n = rx.data_length_code > 8 ? 8 : rx.data_length_code;
      memcpy(r, rx.data, n);
      if (idResp) *idResp = rx.identifier;
      return n;
    }
  }
  return 0;
}

static void bruto(const char *que, const uint8_t r[8], int n) {
  Serial.printf("    %-10s", que);
  for (int i = 0; i < n; i++) Serial.printf(" %02X", r[i]);
  Serial.print("   ");
}

static int16_t i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void describir(uint8_t id) {
  uint8_t r[8];
  uint32_t idr = 0;
  int n;
  Serial.printf("\n  ** Motor en ID %u (0x%03X)\n", id, 0x140 + id);

  if ((n = preguntar(id, 0xB5, r, &idr)) > 0) {
    bruto("0xB5", r, n);
    Serial.print("modelo \"");
    for (int i = 1; i < n; i++) if (r[i] >= 32 && r[i] < 127) Serial.write(r[i]);
    Serial.printf("\" (responde en 0x%03lX)\n", (unsigned long)idr);
  } else {
    Serial.println("    0xB5       sin respuesta (protocolo antiguo: no da el modelo)");
  }
  if ((n = preguntar(id, 0xB2, r, nullptr)) > 0) {
    bruto("0xB2", r, n);
    uint32_t v = r[4] | (r[5] << 8) | (r[6] << 16) | ((uint32_t)r[7] << 24);
    Serial.printf("version %lu\n", (unsigned long)v);
  } else {
    Serial.println("    0xB2       sin respuesta");
  }
  if ((n = preguntar(id, 0x9A, r, nullptr)) > 0) {
    bruto("0x9A", r, n);
    Serial.printf("temp %d C | V4: tension %.1f V, error 0x%04X | V3: tension %.1f V, error 0x%02X\n",
                  (int8_t)r[1], u16(r + 4) * 0.1f, u16(r + 6), u16(r + 3) * 0.1f, r[7]);
  }
  if ((n = preguntar(id, 0x9C, r, nullptr)) > 0) {
    bruto("0x9C", r, n);
    Serial.printf("temp %d C, iq %.2f A, vel %d dps, angulo %d\n",
                  (int8_t)r[1], i16(r + 2) * 0.01f, i16(r + 4), u16(r + 6));
  }
  if ((n = preguntar(id, 0x30, r, nullptr)) > 0) {
    bruto("0x30", r, n);
    Serial.printf("PID: pos %u/%u, vel %u/%u, par %u/%u (Kp/Ki)\n",
                  r[2], r[3], r[4], r[5], r[6], r[7]);
  }
}

static void detectar() {
  Serial.println("\n=== DETECTAR (no mueve) ===");
  int total = 0;
  for (int k = 0; k < 2; k++) {
    bool unMega = (k == 0);
    Serial.printf("\n-- Bus a %s --\n", unMega ? "1 Mbps" : "500 kbps");
    if (!arrancar(unMega)) {
      Serial.println("    no arranca el TWAI");
      continue;
    }
    int aqui = 0;
    for (uint8_t id = 1; id <= ID_MAX; id++) {
      uint8_t r[8];
      if (preguntar(id, 0x9A, r, nullptr) > 0 || preguntar(id, 0x9C, r, nullptr) > 0) {
        describir(id);
        aqui++;
      }
      if (!busVivo()) {
        Serial.println("    bus caido (bus-off): nadie confirma las tramas a esta velocidad");
        break;
      }
    }
    imprimirEstadoBus();
    Serial.printf("    %d motor(es) a esta velocidad\n", aqui);
    total += aqui;
    parar();
  }
  Serial.printf("\nTotal: %d motor(es). %s\n", total,
                total ? "" : "Mira alimentacion, CAN-H/CAN-L y las terminaciones de 120 ohm.");
}

/* ---- Lectura completa del motor 1 (protocolo V4.2, solo lecturas) ---- */

static int32_t i32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static float f32(const uint8_t *p) {
  uint32_t u = (uint32_t)i32(p);
  float f;
  memcpy(&f, &u, 4);
  return f;
}

/* Como preguntar(), con un byte de indice en DATA[1] */
static int preguntarIdx(uint8_t id, uint8_t cmd, uint8_t idx, uint8_t r[8]) {
  if (!busVivo()) return 0;
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {}
  twai_message_t m = {};
  m.identifier = 0x140 + id;
  m.data_length_code = 8;
  m.data[0] = cmd;
  m.data[1] = idx;
  if (twai_transmit(&m, pdMS_TO_TICKS(10)) != ESP_OK) return 0;
  uint32_t fin = millis() + 25;
  while (millis() < fin) {
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK &&
        (rx.identifier == 0x140u + id || rx.identifier == 0x240u + id) &&
        rx.data[0] == cmd) {
      memcpy(r, rx.data, 8);
      return 8;
    }
  }
  return 0;
}

static uint8_t idMotor = 1;

static void leerTodo() {
  uint8_t r[8];
  int n;
  Serial.printf("\n=== LECTURA COMPLETA, motor %u (0x%03X). No mueve ===\n", idMotor, 0x140 + idMotor);
  if (!arrancar(true)) { Serial.println("  no arranca el TWAI"); return; }
  if ((n = preguntar(idMotor, 0xB1, r, nullptr))) {
    bruto("0xB1", r, n); Serial.printf("encendido desde hace %.1f s\n", (uint32_t)i32(r + 4) / 1000.0f);
  }
  if ((n = preguntar(idMotor, 0x70, r, nullptr))) {
    bruto("0x70", r, n);
    Serial.printf("modo %u (1 corriente, 2 velocidad, 3 posicion)\n", r[7]);
  }
  if ((n = preguntar(idMotor, 0x9A, r, nullptr))) {
    bruto("0x9A", r, n);
    uint16_t e = u16(r + 6);
    Serial.printf("temp %d C, freno %u, tension %.1f V, error 0x%04X\n", (int8_t)r[1], r[3],
                  u16(r + 4) * 0.1f, e);
  }
  if ((n = preguntar(idMotor, 0x9C, r, nullptr))) {
    bruto("0x9C", r, n);
    Serial.printf("iq %.2f A, vel %d dps, angulo %d grados\n", i16(r + 2) * 0.01f, i16(r + 4), i16(r + 6));
  }
  if ((n = preguntar(idMotor, 0x9D, r, nullptr))) {
    bruto("0x9D", r, n);
    Serial.printf("fases A %.2f  B %.2f  C %.2f A\n", i16(r + 2) * 0.01f, i16(r + 4) * 0.01f,
                  i16(r + 6) * 0.01f);
  }
  if ((n = preguntar(idMotor, 0x90, r, nullptr))) {
    bruto("0x90", r, n);
    Serial.printf("encoder %u, bruto %u, offset %u (una vuelta)\n", u16(r + 2), u16(r + 4), u16(r + 6));
  }
  if ((n = preguntar(idMotor, 0x94, r, nullptr))) {
    bruto("0x94", r, n); Serial.printf("angulo en la vuelta %.2f grados\n", u16(r + 6) * 0.01f);
  }
  if ((n = preguntar(idMotor, 0x60, r, nullptr))) {
    bruto("0x60", r, n); Serial.printf("encoder multivuelta %ld\n", (long)i32(r + 4));
  }
  if ((n = preguntar(idMotor, 0x61, r, nullptr))) {
    bruto("0x61", r, n); Serial.printf("encoder multivuelta bruto %ld\n", (long)i32(r + 4));
  }
  if ((n = preguntar(idMotor, 0x62, r, nullptr))) {
    bruto("0x62", r, n); Serial.printf("offset de cero %ld\n", (long)i32(r + 4));
  }
  if ((n = preguntar(idMotor, 0x92, r, nullptr))) {
    bruto("0x92", r, n); Serial.printf("angulo multivuelta %.2f grados\n", i32(r + 4) * 0.01f);
  }
  static const struct { uint8_t idx; const char *que; } pid[] = {
      {0x01, "corriente Kp"}, {0x02, "corriente Ki"}, {0x04, "velocidad Kp"},
      {0x05, "velocidad Ki"}, {0x07, "posicion Kp"}, {0x08, "posicion Ki"},
      {0x09, "posicion Kd"}};
  for (auto &p : pid) {
    if (preguntarIdx(idMotor, 0x30, p.idx, r)) Serial.printf("    PID %-13s %g\n", p.que, f32(r + 4));
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (preguntarIdx(idMotor, 0x42, i, r)) Serial.printf("    aceleracion idx %u   %ld dps/s\n", i, (long)i32(r + 4));
  }
  imprimirEstadoBus();
  parar();
}

/* Encoder en vivo 20 s: para girar el eje a mano con el motor apagado */
static void encoderVivo() {
  uint8_t r[8], m[8];
  Serial.println("\n=== ENCODER EN VIVO 20 s (gira el eje a mano). No mueve ===");
  if (!arrancar(true)) { Serial.println("  no arranca el TWAI"); return; }
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    bool a = preguntar(idMotor, 0x90, r, nullptr) > 0;
    bool b = preguntar(idMotor, 0x61, m, nullptr) > 0;
    if (a && b)
      Serial.printf("ENC,%lu,%u,%u,%ld\n", (unsigned long)(millis() - t0), u16(r + 2), u16(r + 4),
                    (long)i32(m + 4));
    else
      Serial.printf("ENC,%lu,sin respuesta\n", (unsigned long)(millis() - t0));
    delay(250);
  }
  parar();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n test_rmd_detectar: d detecta, l lee todo, e encoder en vivo. Nada mueve.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'd' || c == 'D') detectar();
  else if (c == 'l' || c == 'L') leerTodo();
  else if (c == 'e' || c == 'E') encoderVivo();
  else if (c == 'h' || c == 'H')
    Serial.println(" d = detectar IDs 1..32 a 1 Mbps y 500 kbps\n"
                   " l = lectura completa del motor 1 (estado, fases, encoder, PID, aceleracion)\n"
                   " e = encoder en vivo 20 s, para girar el eje a mano");
}
