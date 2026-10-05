/*
 * test_tca9548a.ino - StarCrawler - ENCODERS, PASOS 3 y 4 de 5
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   El multiplexor TCA9548A: el chip que deja leer cuatro AS5600 por
 *   un solo bus, aunque los cuatro tengan la misma direccion (0x36).
 *   Tiene 8 canales; abrir uno conecta el bus a lo que cuelga de el.
 *
 *     PASO 3: el multiplexor SOLO, sin encoders
 *       - que responde en 0x70
 *       - que de verdad cambia de canal (no solo contesta)
 *       - que no se resetea solo (pin RST)
 *       - que con todo cerrado no se cuela nada al bus
 *
 *     PASO 4: el multiplexor con UN encoder, canal a canal
 *       - en que canal aparece el encoder, y solo en ese
 *       - leer el encoder a traves del multiplexor
 *     Mueve el encoder de canal en canal (0, 1, 2, 3) y repite: asi
 *     se prueba cada canal con un sensor que ya sabes que va bien.
 *
 *   RIESGO: NINGUNO. No mueve nada ni escribe en los encoders.
 *
 *   COMO SE CONECTA
 *
 *     ESP32          TCA9548A
 *     3V3    --->    VIN
 *     GND    --->    GND
 *     GPIO21 --->    SDA
 *     GPIO22 --->    SCL
 *     GND    --->    A0, A1, A2   (direccion 0x70)
 *     3V3    --->    RST          (si el modulo no lo trae ya a 3,3 V)
 *
 *     Para el paso 4, un AS5600 en un canal, por ejemplo el 0:
 *     TCA9548A       AS5600
 *     SD0    --->    SDA
 *     SC0    --->    SCL
 *     y el AS5600 con VCC a 3V3, GND a GND y DIR a GND.
 *
 *     Los canales del mux necesitan sus propias pull-ups (4,7 kOhm a
 *     3,3 V en SDx y SCx). Muchos modulos AS5600 las llevan; si el tuyo
 *     no, el canal no funcionara aunque el mux este bien.
 *
 *   Canales en el robot: 0 = FR, 1 = FL, 2 = RR, 3 = RL.
 *
 * ================================================================
 */

#include <Wire.h>

#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define DIR_TCA9548A  0x70
#define DIR_AS5600    0x36

#define REG_STATUS    0x0B
#define REG_RAW_ANGLE 0x0C
#define REG_AGC       0x1A
#define BIT_MH  0x08
#define BIT_ML  0x10
#define BIT_MD  0x20

const char* ORUGA[8] = {"FR", "FL", "RR", "RL", "-", "-", "-", "-"};

/* ---- MULTIPLEXOR ----------------------------------------------- */

bool responde(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

/* El TCA9548A tiene un unico registro: un byte con un bit por canal.
 * Escribirlo abre esos canales; leerlo devuelve los que estan abiertos. */
bool escribirMascara(uint8_t mascara) {
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(mascara);
  return Wire.endTransmission() == 0;
}

bool leerMascara(uint8_t *mascara) {
  if (Wire.requestFrom((int)DIR_TCA9548A, 1) != 1) return false;
  *mascara = Wire.read();
  return true;
}

void cerrarTodo() { escribirMascara(0x00); }

void imprimirHex(uint8_t v) {
  Serial.print(F("0x"));
  if (v < 0x10) Serial.print(F("0"));
  Serial.print(v, HEX);
}

/* ---- AS5600 ---------------------------------------------------- */

bool leerAS(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_AS5600, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool leerEncoder(uint16_t *raw, uint8_t *st, uint8_t *agc) {
  uint8_t b[2];
  if (!leerAS(REG_RAW_ANGLE, b, 2)) return false;
  *raw = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
  return leerAS(REG_STATUS, st, 1) && leerAS(REG_AGC, agc, 1);
}

const char* textoIman(uint8_t st) {
  if (!(st & BIT_MD)) return "SIN IMAN";
  if (st & BIT_ML)    return "DEBIL";
  if (st & BIT_MH)    return "FUERTE";
  return "ok";
}

/* ---- PASO 3: EL MULTIPLEXOR SOLO ------------------------------- */

bool buscar() {
  if (responde(DIR_TCA9548A)) {
    Serial.println(F("  [OK]    Multiplexor en 0x70."));
    return true;
  }
  for (uint8_t d = 0x71; d <= 0x77; d++) {
    if (responde(d)) {
      Serial.print(F("  [ERROR] El multiplexor esta en "));
      imprimirHex(d);
      Serial.println(F(" y no en 0x70."));
      Serial.println(F("          Sus pines A0, A1 y A2 tienen que ir a GND."));
      return false;
    }
  }
  Serial.println(F("  [ERROR] No hay multiplexor en 0x70 ni en 0x71-0x77."));
  Serial.println(F("          Revisa VIN a 3,3 V, GND, SDA al 21 y SCL al 22,"));
  Serial.println(F("          y que RST este a 3,3 V (en bajo, el chip no responde)."));
  Serial.println(F("          test_i2c_escaner te dice si el bus esta sano."));
  return false;
}

/* Contestar a su direccion no prueba que funcione: se escribe cada
 * combinacion de canales y se comprueba que la devuelve igual. */
bool pruebaRegistro() {
  Serial.println(F("  Registro de canales (escribir y releer):"));
  const uint8_t pruebas[] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
                             0x40, 0x80, 0xFF, 0x55, 0xAA};
  int malas = 0;
  for (uint8_t i = 0; i < sizeof(pruebas); i++) {
    uint8_t leida = 0;
    if (!escribirMascara(pruebas[i]) || !leerMascara(&leida) ||
        leida != pruebas[i]) {
      Serial.print(F("    escrito "));
      imprimirHex(pruebas[i]);
      Serial.print(F(", leido "));
      imprimirHex(leida);
      Serial.println(F("  <- NO COINCIDE"));
      malas++;
    }
  }
  cerrarTodo();
  if (malas == 0) {
    Serial.println(F("  [OK]    Cambia de canal y lo recuerda."));
    return true;
  }
  Serial.println(F("  [ERROR] El registro no guarda lo que se escribe: chip"));
  Serial.println(F("          danado, o mala soldadura en SDA/SCL del modulo."));
  return false;
}

/* Si RST esta al aire, el chip se resetea solo de vez en cuando y
 * cierra todos los canales: los encoders "desaparecen" a ratos. */
bool pruebaReset() {
  Serial.println(F("  Estabilidad (3 s con un canal abierto):"));
  escribirMascara(0x05);
  int cambios = 0;
  for (int i = 0; i < 60; i++) {
    uint8_t m = 0;
    if (!leerMascara(&m) || m != 0x05) cambios++;
    delay(50);
  }
  cerrarTodo();
  if (cambios == 0) {
    Serial.println(F("  [OK]    No se resetea solo."));
    return true;
  }
  Serial.print(F("  [ERROR] Se ha perdido el canal "));
  Serial.print(cambios);
  Serial.println(F(" veces de 60."));
  Serial.println(F("          RST al aire o alimentacion que cae: lleva RST a 3,3 V."));
  return false;
}

/* Con todos los canales cerrados, ningun encoder deberia contestar. */
bool pruebaAislamiento() {
  Serial.println(F("  Aislamiento (todo cerrado):"));
  cerrarTodo();
  if (responde(DIR_AS5600)) {
    Serial.println(F("  [ERROR] Un AS5600 contesta con TODOS los canales cerrados:"));
    Serial.println(F("          esta conectado al bus principal, no a un canal."));
    return false;
  }
  Serial.println(F("  [OK]    Con todo cerrado no se cuela ningun encoder."));
  return true;
}

void paso3() {
  Serial.println();
  Serial.println(F("=== PASO 3: EL MULTIPLEXOR SOLO ==="));
  if (!buscar()) { Serial.println(); return; }
  int bien = 0;
  if (pruebaRegistro())    bien++;
  if (pruebaReset())       bien++;
  if (pruebaAislamiento()) bien++;
  Serial.println();
  if (bien == 3) {
    Serial.println(F(">>> El multiplexor funciona. Siguiente: 'c' con un encoder"));
    Serial.println(F("    en un canal (paso 4)."));
  } else {
    Serial.println(F(">>> Arregla lo que falla antes de conectarle encoders."));
  }
  Serial.println();
}

/* ---- PASO 4: CANAL A CANAL ------------------------------------- */

void paso4() {
  Serial.println();
  Serial.println(F("=== PASO 4: QUE HAY DETRAS DE CADA CANAL ==="));
  if (!responde(DIR_TCA9548A)) {
    Serial.println(F("  [ERROR] No responde el multiplexor: haz antes el paso 3 ('m')."));
    Serial.println();
    return;
  }
  int conEncoder = 0;
  int ultimo = -1;
  for (uint8_t c = 0; c < 8; c++) {
    escribirMascara((uint8_t)(1 << c));
    Serial.print(F("  canal "));
    Serial.print(c);
    Serial.print(F(" ("));
    Serial.print(ORUGA[c]);
    Serial.print(F("): "));
    uint16_t raw;
    uint8_t st, agc;
    if (!responde(DIR_AS5600)) {
      Serial.println(F("nada"));
      continue;
    }
    conEncoder++;
    ultimo = c;
    if (leerEncoder(&raw, &st, &agc)) {
      Serial.print(F("AS5600  crudo "));
      Serial.print(raw);
      Serial.print(F("  iman "));
      Serial.print(textoIman(st));
      Serial.print(F("  AGC "));
      Serial.println(agc);
    } else {
      Serial.println(F("AS5600 contesta pero FALLA al leer (cable o pull-ups)"));
    }
  }
  cerrarTodo();
  Serial.println();
  if (conEncoder == 0) {
    Serial.println(F("  [ERROR] Ningun canal tiene encoder. Revisa SDx/SCx del canal,"));
    Serial.println(F("          la alimentacion del encoder y las pull-ups del canal."));
  } else if (conEncoder == 1) {
    Serial.print(F("  [OK]    Un encoder, en el canal "));
    Serial.print(ultimo);
    Serial.println(F(". Para verlo en directo: escribe ese numero."));
  } else {
    Serial.print(F("  Hay "));
    Serial.print(conEncoder);
    Serial.println(F(" encoders. En el robot: canales 0 a 3, uno en cada uno."));
  }
  Serial.println();
}

/* Lectura continua de un encoder a traves del multiplexor */
void verCanal(uint8_t c) {
  Serial.println();
  Serial.print(F("=== CANAL "));
  Serial.print(c);
  Serial.print(F(" ("));
  Serial.print(ORUGA[c]);
  Serial.println(F(") EN CONTINUO (Enter para parar) ==="));
  Serial.println(F("  Gira el iman: el crudo tiene que cambiar."));
  Serial.println(F("  crudo   grados   iman       AGC"));
  while (!Serial.available()) {
    uint16_t raw;
    uint8_t st, agc;
    if (escribirMascara((uint8_t)(1 << c)) && leerEncoder(&raw, &st, &agc)) {
      Serial.print(F("  "));
      if (raw < 1000) Serial.print(F(" "));
      if (raw < 100)  Serial.print(F(" "));
      if (raw < 10)   Serial.print(F(" "));
      Serial.print(raw);
      Serial.print(F("   "));
      Serial.print((float)raw * 0.087890625f, 1);
      Serial.print(F("    "));
      Serial.print(textoIman(st));
      Serial.print(F("        "));
      Serial.println(agc);
    } else {
      Serial.println(F("  NO RESPONDE"));
    }
    delay(200);
  }
  while (Serial.available()) Serial.read();
  cerrarTodo();
  Serial.println(F(">>> Parado."));
  Serial.println();
}

/* ---- MENU ------------------------------------------------------ */

void ayuda() {
  Serial.println();
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe la letra y pulsa Enter)          |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  m   Paso 3: el multiplexor solo                    |"));
  Serial.println(F("|  c   Paso 4: que hay detras de cada canal           |"));
  Serial.println(F("|  0-7 Leer en continuo el encoder de ese canal       |"));
  Serial.println(F("|  h   Esta ayuda                                     |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - ENCODERS, PASOS 3 y 4: MULTIPLEXOR"));
  Serial.println(F("=================================================="));
  Serial.println(F("No mueve nada ni escribe en los encoders."));
  paso3();
  ayuda();
}

void loop() {
  if (!Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  const char c = linea.charAt(0);

  if (c >= '0' && c <= '7') {
    verCanal((uint8_t)(c - '0'));
    return;
  }
  switch (c) {
    case 'm': case 'M': paso3(); break;
    case 'c': case 'C': paso4(); break;
    case 'h': case 'H': case '?': ayuda(); break;
    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
  }
}
