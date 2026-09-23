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

#include <CAN.h>

#define VELOCIDAD_BUS      1000E3
#define CAN_INTER_FRAME_US 250

/* Motor a diagnosticar (peticion). El V3 responde en ID + 0x100. */
#define ID_MOTOR 0x141

#define RMD_CMD_VELOCIDAD 0xA2
#define RMD_CMD_APAGAR    0x80
#define RMD_CMD_ESTADO1   0x9A   /* temp, tension, flags de error */
#define RMD_CMD_ESTADO2   0x9C   /* temp, corriente, velocidad, posicion */
#define RMD_CMD_ANG_MULTI 0x92   /* angulo multivuelta (0.01 grados) */
#define RMD_CMD_ANG_VUELTA 0x94  /* angulo de una vuelta (0.01 grados) */

bool canListo = false;

/* ---- CAN de bajo nivel ------------------------------------------ */

bool enviarTrama(long id, const uint8_t datos[8]) {
  if (!CAN.beginPacket(id)) return false;
  for (int i = 0; i < 8; i++) CAN.write(datos[i]);
  return CAN.endPacket() != 0;
}

int esperarRespuestaCmd(long idPeticion, uint8_t datos[8],
                        unsigned long msLimite, uint8_t cmdEsperado) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    int tam = CAN.parsePacket();
    if (tam > 0) {
      long idResp = CAN.packetId();
      int n = 0;
      while (CAN.available() && n < 8) datos[n++] = CAN.read();
      if ((idResp == idPeticion + 0x100 || idResp == idPeticion) &&
          (cmdEsperado == 0 || (n > 0 && datos[0] == cmdEsperado))) {
        return n;
      }
    }
  }
  return 0;
}

int esperarRespuesta(long idPeticion, uint8_t datos[8], unsigned long msLimite) {
  return esperarRespuestaCmd(idPeticion, datos, msLimite, 0);
}

void vaciarRx() {
  while (CAN.parsePacket() > 0) { while (CAN.available()) CAN.read(); }
}

void apagarMotor() {
  uint8_t t[8] = {RMD_CMD_APAGAR, 0, 0, 0, 0, 0, 0, 0};
  enviarTrama(ID_MOTOR, t);
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
  Serial.println(F("|  p  parada / apagar motor                      |"));
  Serial.println(F("|  h  esta ayuda                                 |"));
  Serial.println(F("+------------------------------------------------+"));
  Serial.println();
}

void estadoCompleto() {
  Serial.println();
  Serial.println(F("=== ESTADO DEL MOTOR ==="));

  vaciarRx();
  uint8_t pet[8] = {RMD_CMD_ESTADO1, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];

  if (!enviarTrama(ID_MOTOR, pet) || esperarRespuestaCmd(ID_MOTOR, r, 100, RMD_CMD_ESTADO1) == 0) {
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
  if (enviarTrama(ID_MOTOR, pet2) && esperarRespuestaCmd(ID_MOTOR, r, 100, RMD_CMD_ESTADO2) > 0) {
    Serial.print(F("  corriente   : ")); Serial.print(i16(r + 2)); Serial.println(F(" (0.01 A/cuenta)"));
    Serial.print(F("  velocidad   : ")); Serial.print(i16(r + 4)); Serial.println(F(" dps"));
    Serial.print(F("  posicion    : ")); Serial.println(i16(r + 6));
  }

  static const uint8_t cmdsAng[2] = {RMD_CMD_ANG_MULTI, RMD_CMD_ANG_VUELTA};
  static const char *nombresAng[2] = {"ang multi  ", "ang vuelta "};
  for (int k = 0; k < 2; k++) {
    vaciarRx();
    uint8_t p[8] = {cmdsAng[k], 0, 0, 0, 0, 0, 0, 0};
    if (enviarTrama(ID_MOTOR, p) && esperarRespuestaCmd(ID_MOTOR, r, 100, cmdsAng[k]) > 0) {
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
    bool enviada = enviarTrama(ID_MOTOR, trama);
    delayMicroseconds(CAN_INTER_FRAME_US);

    uint8_t r[8];
    if (enviada && esperarRespuesta(ID_MOTOR, r, 10) > 0) {
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
  Serial.println(F("=== StarCrawler - diagnostico de UN motor RMD (0x141) ==="));
  Serial.print(F("Bus CAN a 1 Mbps... "));
  if (CAN.begin(VELOCIDAD_BUS)) {
    canListo = true;
    Serial.println(F("OK"));
    apagarMotor();
    Serial.println(F("Motor apagado (arranque seguro)."));
  } else {
    Serial.println(F("FALLO. Revisa el shield."));
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
    case 'p': case 'P': apagarMotor(); Serial.println(F("Motor apagado.")); break;
    case 'h': case 'H': ayuda(); break;
    default: Serial.println(F("? Escribe 'h' para la ayuda.")); break;
  }
}