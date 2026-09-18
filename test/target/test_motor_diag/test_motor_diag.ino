/*
 * test_motor_diag.ino - StarCrawler - diagnostico profundo de UN motor RMD
 * ================================================================
 *
 *   Para cazar fallos intermitentes de UN motor de traccion con datos,
 *   no a ojo: comanda velocidades y registra la telemetria que el motor
 *   devuelve en cada respuesta (velocidad real, corriente, temperatura,
 *   posicion) en lineas CSV por el puerto serie.
 *
 *   *** LOS COMANDOS 'v' Y 'g' MUEVEN EL MOTOR ***
 *   Motor amarrado y eje libre antes de usarlos.
 *
 *   PLACA: Arduino MKR WiFi 1010 + MKR CAN Shield. Un solo motor en el bus.
 *
 *   COMANDOS
 *     s   estado completo: tension, flags de error, temperatura (no mueve)
 *     v   barrido de velocidades con telemetria a 50 Hz (MUEVE, ~24 s)
 *     g   giro sostenido 10 dps / 10 s con telemetria (MUEVE)
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

/* cmdEsperado = 0 acepta cualquier respuesta de ese ID; si se indica, se
 * descartan las respuestas de otro comando (evita leer tramas rezagadas). */
int esperarRespuestaCmd(long idPeticion, uint8_t datos[8],
                        unsigned long msLimite, uint8_t cmdEsperado) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    int tam = CAN.parsePacket();
    if (tam > 0) {
      long idResp = CAN.packetId();
      int n = 0;
      while (CAN.available() && n < 8) datos[n++] = CAN.read();
      /* V3 responde en id+0x100; V2 en el mismo id. Aceptamos ambos. */
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

/* Apaga todo el rango de IDs posibles (por si hay mas de un motor). */
void apagarRango() {
  uint8_t t[8] = {RMD_CMD_APAGAR, 0, 0, 0, 0, 0, 0, 0};
  for (long id = 0x141; id <= 0x148; id++) {
    enviarTrama(id, t);
    delayMicroseconds(CAN_INTER_FRAME_US);
  }
}

/* Busca motores vivos en 0x141..0x148. Devuelve cuantos y rellena ids[]. */
int escanear(long ids[], int maxIds) {
  int n = 0;
  for (long id = 0x141; id <= 0x148 && n < maxIds; id++) {
    vaciarRx();
    uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
    uint8_t r[8];
    if (enviarTrama(id, pet) && esperarRespuesta(id, r, 60) > 0) {
      ids[n++] = id;
      Serial.print(F("  0x")); Serial.print(id, HEX);
      Serial.print(F(" responde  (vel ")); Serial.print(i16(r + 4));
      Serial.print(F(" dps, temp ")); Serial.print((int8_t)r[1]);
      Serial.println(F(" C)"));
    }
    delay(30);
  }
  return n;
}

/* ---- Utilidades ------------------------------------------------- */

String leerLinea(unsigned long msLimite) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    if (Serial.available()) {
      String r = Serial.readStringUntil('\n');
      r.trim();
      r.toUpperCase();
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
  Serial.println(F("|  w  capturar el ARRANQUE (Enter al encender)   |"));
  Serial.println(F("|  v  barrido velocidades + CSV      (MUEVE)     |"));
  Serial.println(F("|  g  10 dps sostenido 10 s + CSV    (MUEVE)     |"));
  Serial.println(F("|  r  bateria de estabilidad ~4.5min (MUEVE)     |"));
  Serial.println(F("|  e  escanear motores en el bus     (no mueve)  |"));
  Serial.println(F("|  x  prueba a DUO: 2 motores a la vez (MUEVE)   |"));
  Serial.println(F("|  o  observacion continua (OJO: cuelga la placa |"));
  Serial.println(F("|     si el bus se queda muerto mucho rato)      |"));
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

  /* 0x9A: [1]=temp int8, [4-5]=tension 0.1V, [6-7]=flags de error */
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
    /* bits segun el manual V3 de MyActuator; verificar con el manual si sale raro */
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

  /* Posicion absoluta del encoder: sobrevive al apagon, sirve para medir
   * cuanto se ha movido el eje durante el "extrano" del arranque. */
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
      Serial.print(F(" grados   [crudo:"));
      for (int i = 0; i < 8; i++) {
        Serial.print(F(" "));
        if (r[i] < 0x10) Serial.print(F("0"));
        Serial.print(r[i], HEX);
      }
      Serial.println(F("]"));
    }
  }
  Serial.println();
}

/* Captura del arranque SIN transmitir con el bus muerto (la leccion del
 * cuelgue: el MCP2515 en bus-off deja endPacket bloqueado para siempre).
 * Aqui no se envia NADA hasta que el usuario pulsa Enter justo al dar
 * alimentacion; entonces se lee a toda velocidad durante 8 s. */
void capturarArranque() {
  Serial.println();
  Serial.println(F("=== CAPTURA DEL ARRANQUE ==="));
  Serial.println(F("1. Con el motor APAGADO, pon la mano en el interruptor."));
  Serial.println(F("2. ENCIENDELO y pulsa Enter INMEDIATAMENTE despues."));
  Serial.println(F("   (hasta que pulses Enter no se envia nada por CAN)"));

  leerLinea(120000UL);   /* espera al Enter, sin tocar el bus */

  Serial.println(F("CSV: DAT,ms,0,vel,corr,temp,pos | BOOT,ms,flags,decivoltios"));
  Serial.println(F("INICIO"));

  unsigned long t0 = millis();
  bool bootLeido = false;
  while (millis() - t0 < 8000) {
    vaciarRx();
    uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
    unsigned long t = millis();
    uint8_t r[8];
    if (enviarTrama(ID_MOTOR, pet) && esperarRespuesta(ID_MOTOR, r, 10) > 0) {
      if (!bootLeido) {
        vaciarRx();
        uint8_t p1[8] = {RMD_CMD_ESTADO1, 0, 0, 0, 0, 0, 0, 0};
        uint8_t rr[8];
        if (enviarTrama(ID_MOTOR, p1) && esperarRespuesta(ID_MOTOR, rr, 20) > 0) {
          uint16_t dv = (uint16_t)rr[4] | ((uint16_t)rr[5] << 8);
          uint16_t fl = (uint16_t)rr[6] | ((uint16_t)rr[7] << 8);
          Serial.print(F("BOOT,"));
          Serial.print(millis() - t0); Serial.print(F(",0x"));
          Serial.print(fl, HEX);       Serial.print(F(","));
          Serial.println(dv);
        }
        bootLeido = true;
      }
      Serial.print(F("DAT,"));
      Serial.print(t - t0);        Serial.print(F(",0,"));
      Serial.print(i16(r + 4));    Serial.print(F(","));
      Serial.print(i16(r + 2));    Serial.print(F(","));
      Serial.print((int8_t)r[1]);  Serial.print(F(","));
      Serial.println(i16(r + 6));
    } else {
      Serial.print(F("MISS,"));
      Serial.print(t - t0);
      Serial.println(F(",0"));
    }
    delay(14);
  }
  Serial.println(F("FIN CAPTURA. Usa 's' para la posicion absoluta de despues."));
  Serial.println();
}

/* Comanda 'dps' durante 'ms' y vuelca una linea CSV por respuesta.
 * Devuelve true si el usuario corto con Enter. */
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
    /* la respuesta al 0xA2 trae temp, corriente, velocidad y posicion */
    if (enviada && esperarRespuesta(ID_MOTOR, r, 10) > 0) {
      Serial.print(F("DAT,"));
      Serial.print(tEnvio - t0);      Serial.print(F(","));
      Serial.print(dps, 0);           Serial.print(F(","));
      Serial.print(i16(r + 4));       Serial.print(F(","));
      Serial.print(i16(r + 2));       Serial.print(F(","));
      Serial.print((int8_t)r[1]);     Serial.print(F(","));
      Serial.println(i16(r + 6));
    } else {
      Serial.print(F("MISS,"));
      Serial.print(tEnvio - t0);      Serial.print(F(","));
      Serial.println(dps, 0);
    }

    /* ritmo ~50 Hz y corte por Enter */
    if (Serial.available()) { while (Serial.available()) Serial.read(); apagarMotor(); Serial.println(F("CORTADO")); return true; }
    delay(18);
  }
  return false;
}

/* Fase a duo: comanda dos motores a la vez (tramas intercaladas con el
 * retardo de 250 us) y vuelca la telemetria de ambos. Es el escenario del
 * fallo historico: varios IDs comandados seguidos.
 * CSV: DUO,ms,id_hex,consigna,vel,corr,temp,pos */
bool faseDuo(long idA, long idB, float dpsA, float dpsB,
             unsigned long ms, unsigned long t0) {
  const long ids[2] = {idA, idB};
  const float dps[2] = {dpsA, dpsB};
  uint8_t trama[2][8];
  for (int k = 0; k < 2; k++) {
    int32_t v = (int32_t)lround(dps[k] * 100.0);
    trama[k][0] = RMD_CMD_VELOCIDAD;
    trama[k][1] = trama[k][2] = trama[k][3] = 0;
    trama[k][4] = (uint8_t)(v & 0xFF);
    trama[k][5] = (uint8_t)((v >> 8) & 0xFF);
    trama[k][6] = (uint8_t)((v >> 16) & 0xFF);
    trama[k][7] = (uint8_t)((v >> 24) & 0xFF);
  }

  unsigned long fin = millis() + ms;
  while (millis() < fin) {
    for (int k = 0; k < 2; k++) {
      vaciarRx();
      unsigned long t = millis();
      uint8_t r[8];
      if (enviarTrama(ids[k], trama[k]) &&
          esperarRespuesta(ids[k], r, 8) > 0) {
        Serial.print(F("DUO,"));
        Serial.print(t - t0);          Serial.print(F(","));
        Serial.print(ids[k], HEX);     Serial.print(F(","));
        Serial.print(dps[k], 0);       Serial.print(F(","));
        Serial.print(i16(r + 4));      Serial.print(F(","));
        Serial.print(i16(r + 2));      Serial.print(F(","));
        Serial.print((int8_t)r[1]);    Serial.print(F(","));
        Serial.println(i16(r + 6));
      } else {
        Serial.print(F("DUOMISS,"));
        Serial.print(t - t0);          Serial.print(F(","));
        Serial.println(ids[k], HEX);
      }
      delayMicroseconds(CAN_INTER_FRAME_US);
    }
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      apagarRango();
      Serial.println(F("CORTADO"));
      return true;
    }
    delay(16);
  }
  return false;
}

/* Prueba a duo: escanea el bus, coge los dos primeros motores que
 * respondan y los mueve a la vez en varias combinaciones. */
void pruebaDuo() {
  Serial.println();
  Serial.println(F("=== PRUEBA A DUO (dos motores a la vez) ==="));
  Serial.println(F("Buscando motores en 0x141..0x148:"));

  long ids[8];
  int n = escanear(ids, 8);
  Serial.print(F("  -> ")); Serial.print(n); Serial.println(F(" motores."));

  if (n < 2) {
    Serial.println(F("Hacen falta al menos 2. Revisa cableado/alimentacion."));
    Serial.println();
    return;
  }

  Serial.println();
  Serial.print(F("Se moveran 0x")); Serial.print(ids[0], HEX);
  Serial.print(F(" y 0x")); Serial.print(ids[1], HEX);
  Serial.println(F(" A LA VEZ:"));
  Serial.println(F("  1) ambos +10 dps (5 s)"));
  Serial.println(F("  2) ambos -10 dps (5 s)"));
  Serial.println(F("  3) opuestos +10/-10 (5 s)"));
  Serial.println(F("  4) ambos +30 dps (5 s)"));
  Serial.println(F("Escribe SI para empezar (otra cosa cancela)."));
  if (leerLinea(30000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  Serial.println(F("CSV: DUO,ms,id_hex,consigna,vel,corr,temp,pos"));
  Serial.println(F("INICIO"));
  unsigned long t0 = millis();

  if (!faseDuo(ids[0], ids[1], 10, 10, 5000, t0) &&
      !faseDuo(ids[0], ids[1], -10, -10, 5000, t0) &&
      !faseDuo(ids[0], ids[1], 10, -10, 5000, t0) &&
      !faseDuo(ids[0], ids[1], 30, 30, 5000, t0)) {
    apagarRango();
    Serial.println(F("FIN. Motores apagados."));
  }
  Serial.println();
}

/* Bateria de estabilidad: inversiones, arranque/parada, micro-velocidad
 * y maraton sostenida. ~4.5 minutos en total, todo con telemetria CSV. */
void pruebaEstabilidad() {
  Serial.println();
  Serial.println(F("=== BATERIA DE ESTABILIDAD (~4.5 min, MUEVE EL MOTOR) ==="));
  Serial.println(F("  A: 20 inversiones bruscas +30/-30 dps      (60 s)"));
  Serial.println(F("  B: 30 ciclos arranque/parada a 20 dps      (45 s)"));
  Serial.println(F("  C: micro-velocidad 1 y 2 dps               (30 s)"));
  Serial.println(F("  D: maraton 25 dps sostenidos               (120 s)"));
  Serial.println(F("Enter en cualquier momento corta y apaga."));
  Serial.println(F("Escribe SI para empezar (otra cosa cancela)."));
  if (leerLinea(30000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  Serial.println(F("CSV: DAT,ms,consigna_dps,vel_dps,corriente_cA,temp_C,pos_raw"));
  Serial.println(F("INICIO"));
  unsigned long t0 = millis();

  Serial.println(F("FASE,A_inversiones"));
  for (int c = 0; c < 20; c++) {
    if (faseVelocidad(30, 1500, t0)) return;
    if (faseVelocidad(-30, 1500, t0)) return;
  }

  Serial.println(F("FASE,B_arranque_parada"));
  for (int c = 0; c < 30; c++) {
    if (faseVelocidad(20, 1000, t0)) return;
    if (faseVelocidad(0, 500, t0)) return;
  }

  Serial.println(F("FASE,C_microvelocidad"));
  if (faseVelocidad(1, 15000, t0)) return;
  if (faseVelocidad(2, 15000, t0)) return;

  Serial.println(F("FASE,D_maraton"));
  if (faseVelocidad(25, 120000, t0)) return;

  apagarMotor();
  Serial.println(F("FIN. Motor apagado."));
  Serial.println();
  estadoCompleto();   /* temperatura, tension y flags tras la paliza */
}

void barridoVelocidades() {
  Serial.println();
  Serial.println(F("=== BARRIDO DE VELOCIDADES (~24 s, MUEVE EL MOTOR) ==="));
  Serial.println(F("Fases: +5 +20 +40 0 -5 -20 -40 0 dps, 3 s cada una."));
  Serial.println(F("Escribe SI para empezar (otra cosa cancela)."));
  if (leerLinea(30000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  Serial.println(F("CSV: DAT,ms,consigna_dps,vel_dps,corriente_cA,temp_C,pos_raw"));
  Serial.println(F("INICIO"));
  const float fases[8] = {5, 20, 40, 0, -5, -20, -40, 0};
  unsigned long t0 = millis();
  for (int f = 0; f < 8; f++) {
    faseVelocidad(fases[f], 3000, t0);
  }
  apagarMotor();
  Serial.println(F("FIN. Motor apagado."));
  Serial.println();
}

void giroSostenido() {
  Serial.println();
  Serial.println(F("=== GIRO SOSTENIDO 10 dps / 10 s (MUEVE EL MOTOR) ==="));
  Serial.println(F("Escribe SI para empezar (otra cosa cancela)."));
  if (leerLinea(30000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  Serial.println(F("CSV: DAT,ms,consigna_dps,vel_dps,corriente_cA,temp_C,pos_raw"));
  Serial.println(F("INICIO"));
  faseVelocidad(10, 10000, millis());
  apagarMotor();
  Serial.println(F("FIN. Motor apagado."));
  Serial.println();
}

/* Observacion pasiva: solo lecturas de estado en bucle. Para capturar
 * que hace el motor al apagarlo y encenderlo ("el extrano" del arranque).
 * Con el motor apagado el envio CAN se queda sin ACK y esto se bloquea
 * dentro de endPacket(): es esperado, se recupera solo al volver la
 * alimentacion. Tras un hueco de >500 ms se leen flags y tension (BOOT). */
void observar() {
  Serial.println();
  Serial.println(F("=== OBSERVACION PASIVA (no manda movimiento) ==="));
  Serial.println(F("Apaga y enciende el motor AHORA. Enter para terminar"));
  Serial.println(F("(el Enter solo funciona con el motor encendido)."));
  Serial.println(F("CSV: DAT,ms,0,vel,corr,temp,pos | BOOT,ms,flags,decivoltios"));
  Serial.println(F("INICIO"));

  unsigned long t0 = millis();
  unsigned long tUltimaResp = millis();
  bool huboHueco = false;

  while (true) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }

    vaciarRx();
    uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
    unsigned long t = millis();
    bool env = enviarTrama(ID_MOTOR, pet);

    uint8_t r[8];
    if (env && esperarRespuesta(ID_MOTOR, r, 10) > 0) {
      if (huboHueco) {
        /* primer contacto tras el apagon: flags y tension del arranque */
        vaciarRx();
        uint8_t p1[8] = {RMD_CMD_ESTADO1, 0, 0, 0, 0, 0, 0, 0};
        uint8_t rr[8];
        if (enviarTrama(ID_MOTOR, p1) && esperarRespuesta(ID_MOTOR, rr, 20) > 0) {
          uint16_t dv = (uint16_t)rr[4] | ((uint16_t)rr[5] << 8);
          uint16_t fl = (uint16_t)rr[6] | ((uint16_t)rr[7] << 8);
          Serial.print(F("BOOT,"));
          Serial.print(millis() - t0); Serial.print(F(",0x"));
          Serial.print(fl, HEX);       Serial.print(F(","));
          Serial.println(dv);
        }
        huboHueco = false;
      }
      Serial.print(F("DAT,"));
      Serial.print(t - t0);        Serial.print(F(",0,"));
      Serial.print(i16(r + 4));    Serial.print(F(","));
      Serial.print(i16(r + 2));    Serial.print(F(","));
      Serial.print((int8_t)r[1]);  Serial.print(F(","));
      Serial.println(i16(r + 6));
      if (t - tUltimaResp > 500) huboHueco = true;  /* por si el BOOT llego tarde */
      tUltimaResp = t;
    } else {
      Serial.print(F("MISS,"));
      Serial.print(t - t0);
      Serial.println(F(",0"));
      if (millis() - tUltimaResp > 500) huboHueco = true;
    }
    delay(14);
  }
  Serial.println(F("FIN OBSERVACION"));
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
    case 'w': case 'W': capturarArranque(); break;
    case 'o': case 'O': observar(); break;
    case 'v': case 'V': barridoVelocidades(); break;
    case 'g': case 'G': giroSostenido(); break;
    case 'r': case 'R': pruebaEstabilidad(); break;
    case 'e': case 'E': { long ids[8]; Serial.println(); escanear(ids, 8); Serial.println(); break; }
    case 'x': case 'X': pruebaDuo(); break;
    case 'p': case 'P': apagarRango(); Serial.println(F("Todos los motores apagados.")); break;
    case 'h': case 'H': ayuda(); break;
    default: Serial.println(F("? Escribe 'h' para la ayuda.")); break;
  }
}
