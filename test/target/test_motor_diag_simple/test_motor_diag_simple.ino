/*
 * test_motor_diag.ino - StarCrawler - diagnostico profundo de UN motor RMD
 * ================================================================
 *
 *   PLACA: Arduino MKR WiFi 1010 + MKR CAN Shield. Un solo motor en el bus.
 *
 *   COMANDOS
 *     s   estado completo: tension, flags de error, temperatura (no mueve)
 *     c   consigna personalizada de velocidad y tiempo (MUEVE)
 *     p   parada / apagar motor
 *     h   ayuda
 *
 *   FORMATO CSV (una linea por respuesta del motor):
 *     DAT,ms,consigna_dps,vel_dps,corriente_cA,temp_C,pos_raw
 *   Peticiones sin respuesta:
 *     MISS,ms,consigna_dps
 */

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/twai.h"
#define PIN_TWAI_TX GPIO_NUM_5
#define PIN_TWAI_RX GPIO_NUM_35
#else
#include <CAN.h>
#endif

#define VELOCIDAD_BUS      1000E3
#define CAN_INTER_FRAME_US 250

/* Motor a diagnosticar (peticion). El V3 responde en ID + 0x100. */
long idMotor = 0x141;   /* se cambia con 'i' */

#define RMD_CMD_VELOCIDAD 0xA2
#define RMD_CMD_APAGAR    0x80
#define RMD_CMD_ESTADO1   0x9A   /* temp, tension, flags de error */
#define RMD_CMD_ESTADO2   0x9C   /* temp, corriente, velocidad, posicion */
#define RMD_CMD_ANG_MULTI 0x92   /* angulo multivuelta (0.01 grados) */
#define RMD_CMD_ANG_VUELTA 0x94  /* angulo de una vuelta (0.01 grados) */

bool canListo = false;

/* ---- CAN de bajo nivel ------------------------------------------ */

#if defined(ARDUINO_ARCH_ESP32)
/* ESP32: TWAI interno + TJA1050, pines de test_can_esp32 */
bool canIniciar() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  return twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
}

/* Con el bus caido no se encola nada: el driver de IDF casca (assert en
 * twai.c). Se inicia la recuperacion y se rearranca en el siguiente envio. */
bool enviarTrama(long id, const uint8_t datos[8]) {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return false;
  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_clear_transmit_queue();
    twai_initiate_recovery();
    return false;
  }
  if (st.state == TWAI_STATE_STOPPED) { twai_start(); return false; }
  if (st.state != TWAI_STATE_RUNNING) return false;
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = 8;
  memcpy(m.data, datos, 8);
  return twai_transmit(&m, pdMS_TO_TICKS(5)) == ESP_OK;
}

int leerTrama(long *id, uint8_t datos[8]) {
  twai_message_t m;
  if (twai_receive(&m, 0) != ESP_OK) return 0;
  *id = m.identifier;
  memcpy(datos, m.data, 8);
  return m.data_length_code;
}
#else
/* MKR WiFi 1010 + MKR CAN Shield */
bool canIniciar() { return CAN.begin(VELOCIDAD_BUS); }

bool enviarTrama(long id, const uint8_t datos[8]) {
  if (!CAN.beginPacket(id)) return false;
  for (int i = 0; i < 8; i++) CAN.write(datos[i]);
  return CAN.endPacket() != 0;
}

int leerTrama(long *id, uint8_t datos[8]) {
  if (CAN.parsePacket() <= 0) return 0;
  *id = CAN.packetId();
  int n = 0;
  while (CAN.available() && n < 8) datos[n++] = CAN.read();
  return n;
}
#endif

int esperarRespuestaCmd(long idPeticion, uint8_t datos[8],
                        unsigned long msLimite, uint8_t cmdEsperado) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    long idResp;
    int n = leerTrama(&idResp, datos);
    if (n > 0 && (idResp == idPeticion + 0x100 || idResp == idPeticion) &&
        (cmdEsperado == 0 || datos[0] == cmdEsperado)) {
      return n;
    }
  }
  return 0;
}

int esperarRespuesta(long idPeticion, uint8_t datos[8], unsigned long msLimite) {
  return esperarRespuestaCmd(idPeticion, datos, msLimite, 0);
}

void vaciarRx() {
  long id;
  uint8_t d[8];
  while (leerTrama(&id, d) > 0) { }
}

void apagarMotor() {
  uint8_t t[8] = {RMD_CMD_APAGAR, 0, 0, 0, 0, 0, 0, 0};
  enviarTrama(idMotor, t);
  delayMicroseconds(CAN_INTER_FRAME_US);
}

/* ---- Utilidades ------------------------------------------------- */

String leerLinea(unsigned long msLimite) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    if (Serial.available()) {
      String r = Serial.readStringUntil('\n');
      r.trim();
      return r;
    }
    delay(10);
  }
  return "";
}

int16_t i16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

/* ---- Comandos --------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------+"));
  Serial.println(F("|  s  estado: tension, flags, pos absoluta       |"));
  Serial.println(F("|  c  consigna personalizada (vel y tiempo) (MUE)|"));
  Serial.println(F("|  f  consigna con angulo fino 0,01 grados (MUE)|"));
  Serial.println(F("|  g  leer ganancias PID del motor (no mueve)    |"));
  Serial.println(F("|  k  Kp/Ki de velocidad en RAM (se pierde al apagar)|"));
  Serial.println(F("|  r  grabar Kp/Ki de velocidad en ROM (pide SI)  |"));
  Serial.println(F("|  i  elegir motor (ID 141-148)                  |"));
  Serial.println(F("|  p  parada / apagar motor                      |"));
  Serial.println(F("|  h  esta ayuda                                 |"));
  Serial.println(F("+------------------------------------------------+"));
  Serial.println();
}

/* ---- Ganancias del motor (solo lectura) ------------------------ */

#define RMD_CMD_LEER_PID   0x30
#define RMD_CMD_PID_RAM    0x31   /* se pierde al apagar el motor */

void volcarHex(const uint8_t *d) {
  for (int i = 0; i < 8; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX);
    Serial.print(' ');
  }
}

float f32(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  float f;
  memcpy(&f, &u, 4);
  return f;
}

/* Prueba el formato V3 (seis uint8 en una trama) y el V4 (un parametro
 * por indice, valor float). Vuelca los bytes: el formato se decide mirando. */
void leerGanancias() {
  Serial.println();
  Serial.println(F("=== GANANCIAS DEL MOTOR (0x30) ==="));
  uint8_t t[8] = {RMD_CMD_LEER_PID, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];
  vaciarRx();
  if (enviarTrama(idMotor, t) && esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0) {
    Serial.print(F("  V3 crudo : ")); volcarHex(r); Serial.println();
    Serial.print(F("  V3 si aplica -> corriente Kp/Ki ")); Serial.print(r[2]); Serial.print('/'); Serial.print(r[3]);
    Serial.print(F("  velocidad Kp/Ki ")); Serial.print(r[4]); Serial.print('/'); Serial.print(r[5]);
    Serial.print(F("  posicion Kp/Ki ")); Serial.print(r[6]); Serial.print('/'); Serial.println(r[7]);
  } else {
    Serial.println(F("  V3: sin respuesta"));
  }
  static const uint8_t idx[6] = {0x01, 0x02, 0x04, 0x05, 0x07, 0x08};
  static const char *nom[6] = {"corriente Kp", "corriente Ki", "velocidad Kp",
                               "velocidad Ki", "posicion Kp ", "posicion Ki "};
  for (int k = 0; k < 6; k++) {
    uint8_t q[8] = {RMD_CMD_LEER_PID, idx[k], 0, 0, 0, 0, 0, 0};
    delay(5);
    vaciarRx();
    Serial.print(F("  V4 idx ")); Serial.print(idx[k]); Serial.print(F(" (")); Serial.print(nom[k]); Serial.print(F("): "));
    if (enviarTrama(idMotor, q) && esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0) {
      volcarHex(r); Serial.print(F(" -> float ")); Serial.println(f32(r + 4), 5);
    } else {
      Serial.println(F("sin respuesta"));
    }
  }
}

/* Escribe Kp/Ki del lazo de velocidad en RAM (0x31, formato V3): se pierden
 * al apagar el motor. Corriente y posicion se reescriben con lo que habia. */
void escribirGananciasVelocidad() {
  Serial.println();
  Serial.println(F("=== GANANCIAS DE VELOCIDAD EN RAM (0x31) ==="));
  uint8_t t[8] = {RMD_CMD_LEER_PID, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];
  vaciarRx();
  if (!(enviarTrama(idMotor, t) &&
        esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0)) {
    Serial.println(F("  No puedo leer las actuales: no escribo nada."));
    return;
  }
  Serial.print(F("  actuales velocidad Kp/Ki ")); Serial.print(r[4]);
  Serial.print('/'); Serial.println(r[5]);
  Serial.println(F("Kp de velocidad (0-255):"));
  String sKp = leerLinea(30000UL);
  if (sKp.length() == 0) { Serial.println(F("Cancelado.")); return; }
  Serial.println(F("Ki de velocidad (0-255):"));
  String sKi = leerLinea(30000UL);
  if (sKi.length() == 0) { Serial.println(F("Cancelado.")); return; }
  int kp = constrain(sKp.toInt(), 0, 255);
  int ki = constrain(sKi.toInt(), 0, 255);
  uint8_t w[8] = {RMD_CMD_PID_RAM, 0, r[2], r[3], (uint8_t)kp, (uint8_t)ki, r[6], r[7]};
  vaciarRx();
  if (!(enviarTrama(idMotor, w) &&
        esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_PID_RAM) > 0)) {
    Serial.println(F("  SIN respuesta al escribir."));
    return;
  }
  delay(5);
  vaciarRx();
  if (enviarTrama(idMotor, t) &&
      esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0) {
    Serial.print(F("  ESCRITO. Ahora velocidad Kp/Ki ")); Serial.print(r[4]);
    Serial.print('/'); Serial.println(r[5]);
  }
}

#define RMD_CMD_PID_ROM    0x32   /* permanente: sobrevive al apagado */

/* Graba en ROM las seis ganancias: las de velocidad nuevas, corriente y
 * posicion las que ya tiene. Pide confirmacion porque no se deshace sola. */
void grabarGananciasROM() {
  Serial.println();
  Serial.println(F("=== GRABAR GANANCIAS EN ROM (0x32) ==="));
  uint8_t t[8] = {RMD_CMD_LEER_PID, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];
  vaciarRx();
  if (!(enviarTrama(idMotor, t) &&
        esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0)) {
    Serial.println(F("  No puedo leer las actuales: no grabo nada."));
    return;
  }
  Serial.print(F("  motor 0x")); Serial.print(idMotor, HEX);
  Serial.print(F("  actuales velocidad Kp/Ki ")); Serial.print(r[4]);
  Serial.print('/'); Serial.println(r[5]);
  Serial.println(F("Kp de velocidad (0-255):"));
  String sKp = leerLinea(30000UL);
  if (sKp.length() == 0) { Serial.println(F("Cancelado.")); return; }
  Serial.println(F("Ki de velocidad (0-255):"));
  String sKi = leerLinea(30000UL);
  if (sKi.length() == 0) { Serial.println(F("Cancelado.")); return; }
  int kp = constrain(sKp.toInt(), 0, 255);
  int ki = constrain(sKi.toInt(), 0, 255);
  Serial.print(F("Grabar en ROM velocidad Kp/Ki ")); Serial.print(kp);
  Serial.print('/'); Serial.print(ki);
  Serial.println(F("? Escribe SI para confirmar."));
  if (leerLinea(30000UL) != "SI") { Serial.println(F("Cancelado, no se graba.")); return; }
  uint8_t w[8] = {RMD_CMD_PID_ROM, 0, r[2], r[3], (uint8_t)kp, (uint8_t)ki, r[6], r[7]};
  vaciarRx();
  if (!(enviarTrama(idMotor, w) &&
        esperarRespuestaCmd(idMotor, r, 200, RMD_CMD_PID_ROM) > 0)) {
    Serial.println(F("  SIN respuesta al grabar."));
    return;
  }
  delay(20);
  vaciarRx();
  if (enviarTrama(idMotor, t) &&
      esperarRespuestaCmd(idMotor, r, 30, RMD_CMD_LEER_PID) > 0) {
    Serial.print(F("  GRABADO EN ROM. Ahora velocidad Kp/Ki ")); Serial.print(r[4]);
    Serial.print('/'); Serial.println(r[5]);
  }
}

void elegirMotor() {
  Serial.print(F("Motor actual 0x")); Serial.print(idMotor, HEX);
  Serial.println(F(". Nuevo ID en hexadecimal (141-148):"));
  String s = leerLinea(30000UL);
  if (s.length() == 0) { Serial.println(F("Cancelado.")); return; }
  long id = strtol(s.c_str(), NULL, 16);
  if (id < 0x141 || id > 0x148) { Serial.println(F("Fuera de rango, sin cambios.")); return; }
  idMotor = id;
  Serial.print(F("  MOTOR 0x")); Serial.println(idMotor, HEX);
}

void estadoCompleto() {
  Serial.println();
  Serial.println(F("=== ESTADO DEL MOTOR ==="));

  vaciarRx();
  uint8_t pet[8] = {RMD_CMD_ESTADO1, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];

  if (!enviarTrama(idMotor, pet) || esperarRespuestaCmd(idMotor, r, 100, RMD_CMD_ESTADO1) == 0) {
    Serial.println(F("Sin respuesta al 0x9A."));
    return;
  }

  int8_t   temp    = (int8_t)r[1];
  uint16_t decivolt = (uint16_t)r[4] | ((uint16_t)r[5] << 8);
  uint16_t flags   = (uint16_t)r[6] | ((uint16_t)r[7] << 8);

  Serial.print(F("  temperatura : ")); Serial.print(temp); Serial.println(F(" C"));
  Serial.print(F("  tension     : ")); Serial.print(decivolt / 10.0f, 1); Serial.println(F(" V"));
  Serial.print(F("  flags error : 0x"));
  if (flags < 0x1000) Serial.print(F("0"));
  if (flags < 0x100)  Serial.print(F("0"));
  if (flags < 0x10)   Serial.print(F("0"));
  Serial.println(flags, HEX);

  if (flags == 0) {
    Serial.println(F("                (sin errores activos)"));
  } else {
    if (flags & 0x0002) Serial.println(F("                - motor stall (bloqueo)"));
    if (flags & 0x0004) Serial.println(F("                - baja tension"));
    if (flags & 0x0008) Serial.println(F("                - sobretension"));
    if (flags & 0x0010) Serial.println(F("                - sobrecorriente"));
    if (flags & 0x0100) Serial.println(F("                - exceso de velocidad"));
    if (flags & 0x1000) Serial.println(F("                - sobretemperatura"));
    if (flags & 0x2000) Serial.println(F("                - error calibracion encoder"));
  }

  vaciarRx();
  uint8_t pet2[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
  if (enviarTrama(idMotor, pet2) && esperarRespuestaCmd(idMotor, r, 100, RMD_CMD_ESTADO2) > 0) {
    Serial.print(F("  corriente   : ")); Serial.print(i16(r + 2)); Serial.println(F(" (0.01 A/cuenta)"));
    Serial.print(F("  velocidad   : ")); Serial.print(i16(r + 4)); Serial.println(F(" dps"));
    Serial.print(F("  posicion    : ")); Serial.println(i16(r + 6));
  }

  static const uint8_t cmdsAng[2] = {RMD_CMD_ANG_MULTI, RMD_CMD_ANG_VUELTA};
  static const char *nombresAng[2] = {"ang multi  ", "ang vuelta "};
  for (int k = 0; k < 2; k++) {
    vaciarRx();
    uint8_t p[8] = {cmdsAng[k], 0, 0, 0, 0, 0, 0, 0};
    if (enviarTrama(idMotor, p) && esperarRespuestaCmd(idMotor, r, 100, cmdsAng[k]) > 0) {
      int32_t v = (int32_t)((uint32_t)r[4] | ((uint32_t)r[5] << 8) |
                            ((uint32_t)r[6] << 16) | ((uint32_t)r[7] << 24));
      Serial.print(F("  ")); Serial.print(nombresAng[k]);
      Serial.print(F(": ")); Serial.print(v / 100.0f, 2);
      Serial.println(F(" grados"));
    }
  }
  Serial.println();
}

bool faseVelocidad(float dps, unsigned long ms, unsigned long t0) {
  uint8_t trama[8];
  int32_t v = (int32_t)lround(dps * 100.0);
  trama[0] = RMD_CMD_VELOCIDAD;
  trama[1] = trama[2] = trama[3] = 0;
  trama[4] = (uint8_t)(v & 0xFF);
  trama[5] = (uint8_t)((v >> 8) & 0xFF);
  trama[6] = (uint8_t)((v >> 16) & 0xFF);
  trama[7] = (uint8_t)((v >> 24) & 0xFF);

  unsigned long fin = millis() + ms;
  while (millis() < fin) {
    vaciarRx();
    unsigned long tEnvio = millis();
    bool enviada = enviarTrama(idMotor, trama);
    delayMicroseconds(CAN_INTER_FRAME_US);

    uint8_t r[8];
    if (enviada && esperarRespuesta(idMotor, r, 10) > 0) {
      Serial.print(F("DAT,"));
      Serial.print(tEnvio - t0);      Serial.print(F(","));
      Serial.print(dps, 1);           Serial.print(F(","));
      Serial.print(i16(r + 4));       Serial.print(F(","));
      Serial.print(i16(r + 2));       Serial.print(F(","));
      Serial.print((int8_t)r[1]);     Serial.print(F(","));
      Serial.println(i16(r + 6));
    } else {
      Serial.print(F("MISS,"));
      Serial.print(tEnvio - t0);      Serial.print(F(","));
      Serial.println(dps, 1);
    }

    if (Serial.available()) { 
      while (Serial.available()) Serial.read(); 
      apagarMotor(); 
      Serial.println(F("CORTADO")); 
      return true; 
    }
    delay(18);
  }
  return false;
}

/* Como faseVelocidad, pero tras cada consigna pide el angulo multivuelta
 * (0x92, 0,01 grados): con la posicion de 0xA2 (1 grado) no se ven los
 * tirones de menos de un grado. */
bool faseFina(float dps, unsigned long ms, unsigned long t0) {
  uint8_t trama[8];
  int32_t v = (int32_t)lround(dps * 100.0);
  trama[0] = RMD_CMD_VELOCIDAD;
  trama[1] = trama[2] = trama[3] = 0;
  trama[4] = (uint8_t)(v & 0xFF);
  trama[5] = (uint8_t)((v >> 8) & 0xFF);
  trama[6] = (uint8_t)((v >> 16) & 0xFF);
  trama[7] = (uint8_t)((v >> 24) & 0xFF);
  const uint8_t pideAng[8] = {RMD_CMD_ANG_MULTI, 0, 0, 0, 0, 0, 0, 0};

  unsigned long fin = millis() + ms;
  while (millis() < fin) {
    vaciarRx();
    unsigned long tEnvio = millis();
    uint8_t r[8], a[8];
    bool okVel = enviarTrama(idMotor, trama) &&
                 esperarRespuestaCmd(idMotor, r, 10, RMD_CMD_VELOCIDAD) > 0;
    delayMicroseconds(CAN_INTER_FRAME_US);
    unsigned long tAng = millis();
    bool okAng = enviarTrama(idMotor, pideAng) &&
                 esperarRespuestaCmd(idMotor, a, 10, RMD_CMD_ANG_MULTI) > 0;
    if (okVel && okAng) {
      int32_t ang = (int32_t)((uint32_t)a[4] | ((uint32_t)a[5] << 8) |
                              ((uint32_t)a[6] << 16) | ((uint32_t)a[7] << 24));
      Serial.print(F("FIN,"));
      Serial.print(tEnvio - t0);   Serial.print(F(","));
      Serial.print(tAng - t0);     Serial.print(F(","));
      Serial.print(i16(r + 4));    Serial.print(F(","));
      Serial.print(i16(r + 2));    Serial.print(F(","));
      Serial.println(ang);
    } else {
      Serial.print(F("MISS,"));
      Serial.println(tEnvio - t0);
    }
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      apagarMotor();
      Serial.println(F("CORTADO"));
      return true;
    }
    delay(15);
  }
  return false;
}

void consignaFina() {
  Serial.println();
  Serial.println(F("=== CONSIGNA CON ANGULO FINO ==="));
  Serial.println(F("Velocidad en dps:"));
  String sVel = leerLinea(30000UL);
  if (sVel.length() == 0) { Serial.println(F("Cancelado.")); return; }
  Serial.println(F("Duracion en segundos:"));
  String sT = leerLinea(30000UL);
  if (sT.length() == 0) { Serial.println(F("Cancelado.")); return; }
  float dps = sVel.toFloat();
  unsigned long s = sT.toInt();
  Serial.println(F("CSV: FIN,ms_vel,ms_ang,vel_dps,corriente_cA,ang_centigrados"));
  Serial.println(F("INICIO"));
  faseFina(dps, s * 1000UL, millis());
  apagarMotor();
  Serial.println(F("FIN. Motor apagado."));
}

void consignaPersonalizada() {
  Serial.println();
  Serial.println(F("=== CONSIGNA PERSONALIZADA ==="));
  
  Serial.println(F("Introduce la velocidad deseada en dps (ej: 15.5 o -20):"));
  String sVel = leerLinea(30000UL);
  if (sVel.length() == 0) { Serial.println(F("Cancelado.")); return; }
  float dpsUser = sVel.toFloat();

  Serial.println(F("Introduce la duracion en segundos (ej: 5):"));
  String sTiempo = leerLinea(30000UL);
  if (sTiempo.length() == 0) { Serial.println(F("Cancelado.")); return; }
  unsigned long segundosUser = sTiempo.toInt();
  unsigned long msUser = segundosUser * 1000UL;

  Serial.print(F("Ejecutando "));
  Serial.print(dpsUser);
  Serial.print(F(" dps durante "));
  Serial.print(segundosUser);
  Serial.println(F(" segundos..."));
  
  Serial.println(F("CSV: DAT,ms,consigna_dps,vel_dps,corriente_cA,temp_C,pos_raw"));
  Serial.println(F("INICIO"));

  unsigned long t0 = millis();
  faseVelocidad(dpsUser, msUser, t0);

  apagarMotor();
  Serial.println(F("FIN. Motor apagado."));
  Serial.println();
}

/* ---- Arranque --------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) { }
  delay(300);

  Serial.println();
  Serial.println(F("=== StarCrawler - diagnostico de UN motor RMD (0x141 por defecto) ==="));
  Serial.print(F("Bus CAN a 1 Mbps... "));
  if (canIniciar()) {
    canListo = true;
    Serial.println(F("OK"));
    apagarMotor();
    Serial.println(F("Motor apagado (arranque seguro)."));
  } else {
    Serial.println(F("FALLO. Revisa el shield o el transceptor."));
    return;
  }
  ayuda();
}

void loop() {
  if (!canListo || !Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  switch (linea.charAt(0)) {
    case 's': case 'S': estadoCompleto(); break;
    case 'c': case 'C': consignaPersonalizada(); break;
    case 'f': case 'F': consignaFina(); break;
    case 'g': case 'G': leerGanancias(); break;
    case 'k': case 'K': escribirGananciasVelocidad(); break;
    case 'r': case 'R': grabarGananciasROM(); break;
    case 'i': case 'I': elegirMotor(); break;
    case 'p': case 'P': apagarMotor(); Serial.println(F("Motor apagado.")); break;
    case 'h': case 'H': ayuda(); break;
    default: Serial.println(F("? Escribe 'h' para la ayuda.")); break;
  }
}