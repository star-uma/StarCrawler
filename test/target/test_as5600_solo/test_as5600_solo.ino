/*
 * test_as5600_solo.ino - StarCrawler - ENCODERS, PASO 2 de 5
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   UN solo encoder AS5600, conectado directo al ESP32, SIN el
 *   multiplexor. Si aqui funciona y en el robot no, el problema no es
 *   el sensor: es el multiplexor, el cable o el montaje.
 *
 *   Mira lo que el chip sabe de si mismo:
 *     - si ve el iman, y si esta demasiado lejos o demasiado cerca
 *     - cuanto tiene que amplificar (AGC): dice si el iman esta bien
 *     - el ruido de la lectura con el eje quieto
 *     - una vuelta completa: si el iman esta centrado
 *     - el sentido: si el valor sube o baja al girar
 *
 *   RIESGO: NINGUNO. Solo lee: no escribe ningun registro del chip
 *   (el AS5600 tiene memoria que se graba una vez y para siempre;
 *   este sketch no la toca).
 *
 *   COMO SE CONECTA (el multiplexor NO va conectado)
 *
 *     ESP32          AS5600
 *     3V3    --->    VCC
 *     GND    --->    GND
 *     GPIO21 --->    SDA
 *     GPIO22 --->    SCL
 *     GND    --->    DIR      (al aire NO: el sentido cambiaria solo)
 *                    OUT      sin conectar
 *                    GPO/PGO  sin conectar
 *
 *   El iman: de magnetizacion DIAMETRAL (los polos a los lados, no
 *   arriba y abajo), centrado sobre el chip y a unos 0,5-3 mm.
 *
 *   COMO SE USA
 *     1. Compila y sube el sketch (ver test/target/README.md)
 *     2. Monitor Serie a 115200 baudios. 'h' = ayuda
 *
 * ================================================================
 */

#include <Wire.h>

#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define DIR_AS5600    0x36

/* Registros del AS5600 (datasheet, seccion "Register Map") */
#define REG_ZMCO      0x00   /* cuantas veces se ha grabado la memoria */
#define REG_ZPOS      0x01   /* posicion de cero programada (2 bytes)  */
#define REG_MPOS      0x03   /* posicion maxima programada (2 bytes)   */
#define REG_MANG      0x05   /* angulo maximo programado (2 bytes)     */
#define REG_CONF      0x07   /* configuracion (2 bytes)                */
#define REG_STATUS    0x0B   /* estado del iman                        */
#define REG_RAW_ANGLE 0x0C   /* angulo crudo, 0..4095 (2 bytes)        */
#define REG_AGC       0x1A   /* ganancia automatica                    */
#define REG_MAGNITUDE 0x1B   /* intensidad del campo (2 bytes)         */

#define BIT_MH  0x08         /* iman demasiado fuerte */
#define BIT_ML  0x10         /* iman demasiado debil  */
#define BIT_MD  0x20         /* iman detectado        */

/* ---- LECTURA --------------------------------------------------- */

bool leerRegistros(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_AS5600, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

/* Los valores de 12 bits ocupan dos registros: alto (4 bits) y bajo */
bool leer12(uint8_t reg, uint16_t *v) {
  uint8_t b[2];
  if (!leerRegistros(reg, b, 2)) return false;
  *v = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
  return true;
}

bool leer8(uint8_t reg, uint8_t *v) {
  return leerRegistros(reg, v, 1);
}

/* Diferencia con signo entre dos lecturas crudas, por el camino corto:
 * de 4090 a 5 no son -4085 cuentas, son +11. */
int difCrudo(uint16_t de, uint16_t a) {
  int d = (int)a - (int)de;
  if (d > 2048)  d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

float aGrados(int cuentas) { return (float)cuentas * 0.087890625f; }

bool presente() {
  Wire.beginTransmission(DIR_AS5600);
  return Wire.endTransmission() == 0;
}

/* Texto corto del estado del iman, para las tablas */
const char* textoIman(uint8_t st) {
  if (!(st & BIT_MD)) return "SIN IMAN";
  if (st & BIT_ML)    return "DEBIL";
  if (st & BIT_MH)    return "FUERTE";
  return "ok";
}

/* Explicacion larga del estado del iman */
void explicarIman(uint8_t st) {
  if (!(st & BIT_MD)) {
    Serial.println(F("  [ERROR] El chip NO detecta el iman."));
    Serial.println(F("          Falta el iman, esta muy lejos (>3 mm), o no es"));
    Serial.println(F("          de magnetizacion diametral."));
  } else if (st & BIT_ML) {
    Serial.println(F("  [AVISO] Iman DEMASIADO DEBIL: acercalo al chip."));
  } else if (st & BIT_MH) {
    Serial.println(F("  [AVISO] Iman DEMASIADO FUERTE: alejalo un poco."));
  } else {
    Serial.println(F("  [OK]    Iman detectado y a buena distancia."));
  }
}

void explicarAGC(uint8_t agc) {
  Serial.print(F("  AGC = "));
  Serial.print(agc);
  if (agc > 128) {
    Serial.println(F(" de 255 (el chip esta en modo 5 V)"));
  } else {
    Serial.println(F(" de 128 (modo 3,3 V, el del robot)"));
  }
  Serial.println(F("        Es la ganancia que necesita: baja = iman cerca,"));
  Serial.println(F("        alta = iman lejos. Lo ideal, por la mitad."));
}

/* ---- COMANDOS -------------------------------------------------- */

void estado() {
  Serial.println();
  Serial.println(F("=== ESTADO DEL AS5600 ==="));
  if (!presente()) {
    Serial.println(F("  [ERROR] No responde en 0x36."));
    Serial.println(F("          Revisa VCC a 3,3 V, GND, SDA al 21 y SCL al 22."));
    Serial.println(F("          test_i2c_escaner te dice si el bus esta sano."));
    Serial.println();
    return;
  }
  Serial.println(F("  [OK]    Responde en 0x36."));

  uint8_t st = 0, agc = 0, zmco = 0;
  uint16_t raw = 0, mag = 0, zpos = 0, mpos = 0, mang = 0;
  uint8_t conf[2] = {0, 0};
  bool ok = leer8(REG_STATUS, &st) && leer8(REG_AGC, &agc) &&
            leer12(REG_RAW_ANGLE, &raw) && leer12(REG_MAGNITUDE, &mag) &&
            leer8(REG_ZMCO, &zmco) && leer12(REG_ZPOS, &zpos) &&
            leer12(REG_MPOS, &mpos) && leer12(REG_MANG, &mang) &&
            leerRegistros(REG_CONF, conf, 2);
  if (!ok) {
    Serial.println(F("  [ERROR] Responde a su direccion pero falla al leer."));
    Serial.println(F("          Suele ser un cable que hace mal contacto."));
    Serial.println();
    return;
  }

  Serial.println();
  explicarIman(st);
  explicarAGC(agc);
  Serial.print(F("  Magnitud del campo = "));
  Serial.println(mag);
  Serial.print(F("  Angulo crudo = "));
  Serial.print(raw);
  Serial.print(F("  ("));
  Serial.print(aGrados(raw), 1);
  Serial.println(F(" grados)"));

  /* La memoria grabable: no deberia haberla tocado nadie */
  Serial.println();
  Serial.print(F("  Veces grabado (ZMCO) = "));
  Serial.println(zmco & 0x03);
  if ((zmco & 0x03) != 0 || zpos || mpos || mang) {
    Serial.println(F("  [AVISO] Alguien ha programado este chip (cero o rango)."));
    Serial.println(F("          No afecta: el firmware lee el angulo CRUDO, que"));
    Serial.println(F("          no depende de eso. Pero conviene saberlo."));
  }
  if (conf[0] || conf[1]) {
    Serial.print(F("  [AVISO] Configuracion distinta de la de fabrica: 0x"));
    Serial.print(conf[0], HEX);
    if (conf[1] < 0x10) Serial.print(F("0"));
    Serial.println(conf[1], HEX);
  }
  Serial.println();
}

void continuo() {
  Serial.println();
  Serial.println(F("=== LECTURA CONTINUA (Enter para parar) ==="));
  Serial.println(F("  crudo   grados   iman       AGC"));
  while (!Serial.available()) {
    uint16_t raw;
    uint8_t st, agc;
    if (leer12(REG_RAW_ANGLE, &raw) && leer8(REG_STATUS, &st) &&
        leer8(REG_AGC, &agc)) {
      Serial.print(F("  "));
      if (raw < 1000) Serial.print(F(" "));
      if (raw < 100)  Serial.print(F(" "));
      if (raw < 10)   Serial.print(F(" "));
      Serial.print(raw);
      Serial.print(F("   "));
      Serial.print(aGrados(raw), 1);
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
  Serial.println(F(">>> Parado."));
  Serial.println();
}

/* Con el eje QUIETO, cuanto bailan las lecturas. */
void ruido() {
  Serial.println();
  Serial.println(F("=== RUIDO CON EL EJE QUIETO ==="));
  Serial.println(F("  No toques el eje ni la mesa: 200 lecturas en 2 s..."));
  uint16_t primero;
  if (!leer12(REG_RAW_ANGLE, &primero)) {
    Serial.println(F("  [ERROR] No responde."));
    Serial.println();
    return;
  }
  int minimo = 0, maximo = 0, fallos = 0;
  long suma = 0;
  int n = 0;
  for (int i = 0; i < 200; i++) {
    uint16_t raw;
    if (leer12(REG_RAW_ANGLE, &raw)) {
      int d = difCrudo(primero, raw);
      if (d < minimo) minimo = d;
      if (d > maximo) maximo = d;
      suma += d;
      n++;
    } else {
      fallos++;
    }
    delay(10);
  }
  const int pp = maximo - minimo;
  Serial.print(F("  Pico a pico: "));
  Serial.print(pp);
  Serial.print(F(" cuentas = "));
  Serial.print(aGrados(pp), 2);
  Serial.println(F(" grados"));
  if (n > 0) {
    Serial.print(F("  Media: "));
    Serial.print(aGrados((int)(suma / n)), 2);
    Serial.println(F(" grados respecto a la primera lectura"));
  }
  if (fallos) {
    Serial.print(F("  [AVISO] "));
    Serial.print(fallos);
    Serial.println(F(" lecturas fallidas: contacto intermitente en el cable."));
  }
  if (pp <= 4) {
    Serial.println(F("  [OK]    Muy estable."));
  } else if (pp <= 10) {
    Serial.println(F("  [OK]    Aceptable (menos de 1 grado)."));
  } else {
    Serial.println(F("  [AVISO] Baila demasiado. Mira el iman con 's' (lejos o"));
    Serial.println(F("          debil), que el eje no tenga holgura, y que no haya"));
    Serial.println(F("          un motor o un iman cerca que lo perturbe."));
  }
  Serial.println();
}

/* Una vuelta completa a mano: dice si el iman esta centrado. */
void vuelta() {
  Serial.println();
  Serial.println(F("=== UNA VUELTA COMPLETA ==="));
  Serial.println(F("  Gira el eje DESPACIO, al menos una vuelta entera, y"));
  Serial.println(F("  pulsa Enter al acabar (maximo 2 min)."));
  Serial.println();

  bool visto[64];
  for (int i = 0; i < 64; i++) visto[i] = false;
  uint16_t anterior;
  if (!leer12(REG_RAW_ANGLE, &anterior)) {
    Serial.println(F("  [ERROR] No responde."));
    Serial.println();
    return;
  }
  long recorrido = 0;
  int saltoMax = 0, perdidas = 0, fallos = 0;
  uint8_t agcMin = 255, agcMax = 0;
  const unsigned long fin = millis() + 120000UL;

  while (!Serial.available() && millis() < fin) {
    uint16_t raw;
    uint8_t st, agc;
    if (leer12(REG_RAW_ANGLE, &raw) && leer8(REG_STATUS, &st) &&
        leer8(REG_AGC, &agc)) {
      int d = difCrudo(anterior, raw);
      recorrido += d;
      if (abs(d) > saltoMax) saltoMax = abs(d);
      anterior = raw;
      visto[raw >> 6] = true;
      if (agc < agcMin) agcMin = agc;
      if (agc > agcMax) agcMax = agc;
      if (!(st & BIT_MD) || (st & (BIT_ML | BIT_MH))) perdidas++;
    } else {
      fallos++;
    }
    delay(10);
  }
  while (Serial.available()) Serial.read();

  int zonas = 0;
  for (int i = 0; i < 64; i++) if (visto[i]) zonas++;
  const float vueltas = aGrados((int)labs(recorrido)) / 360.0f;

  Serial.print(F("  Vueltas dadas: "));
  Serial.println(vueltas, 2);
  Serial.print(F("  Zonas del circulo con lectura: "));
  Serial.print(zonas);
  Serial.println(F(" de 64"));
  Serial.print(F("  AGC durante la vuelta: de "));
  Serial.print(agcMin);
  Serial.print(F(" a "));
  Serial.println(agcMax);
  Serial.println();

  if (vueltas < 1.0f) {
    Serial.println(F("  [AVISO] No has llegado a dar la vuelta entera: repite."));
  } else if (zonas < 64) {
    Serial.println(F("  [ERROR] Hay zonas del circulo que nunca se leen."));
  } else {
    Serial.println(F("  [OK]    Se lee el circulo entero."));
  }
  /* Con el iman centrado, el campo es el mismo en toda la vuelta y la
   * ganancia no se mueve. Si baila, el iman esta descentrado o el eje
   * cabecea. 20 es ~15 % del rango de 3,3 V. */
  if (agcMax - agcMin > 20) {
    Serial.println(F("  [AVISO] La ganancia cambia mucho durante la vuelta: el"));
    Serial.println(F("          iman esta descentrado o el eje cabecea."));
  } else {
    Serial.println(F("  [OK]    Ganancia estable: el iman esta centrado."));
  }
  if (perdidas) {
    Serial.print(F("  [AVISO] "));
    Serial.print(perdidas);
    Serial.println(F(" lecturas con el iman fuera de rango durante la vuelta."));
  }
  /* A mano y despacio salen unas pocas cuentas cada 10 ms. Un salto de
   * mas de 100 (~9 grados) es una lectura mala, no un giro. */
  if (saltoMax > 100) {
    Serial.print(F("  [AVISO] Salto de "));
    Serial.print(aGrados(saltoMax), 1);
    Serial.println(F(" grados entre dos lecturas: lectura corrupta."));
  }
  if (fallos) {
    Serial.print(F("  [AVISO] "));
    Serial.print(fallos);
    Serial.println(F(" lecturas fallidas."));
  }
  Serial.println();
}

/* Si el valor sube o baja al girar en un sentido conocido. */
void sentido() {
  Serial.println();
  Serial.println(F("=== SENTIDO DE GIRO ==="));
  uint16_t antes;
  if (!leer12(REG_RAW_ANGLE, &antes)) {
    Serial.println(F("  [ERROR] No responde."));
    Serial.println();
    return;
  }
  Serial.println(F("  Mirando el iman de frente, gira el eje un cuarto de"));
  Serial.println(F("  vuelta en sentido HORARIO y pulsa Enter."));
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  uint16_t despues;
  if (!leer12(REG_RAW_ANGLE, &despues)) {
    Serial.println(F("  [ERROR] No responde."));
    Serial.println();
    return;
  }
  const int d = difCrudo(antes, despues);
  Serial.print(F("  Cambio: "));
  Serial.print(aGrados(d), 1);
  Serial.println(F(" grados"));
  if (abs(d) < 34) {                       /* < 3 grados */
    Serial.println(F("  [AVISO] Casi no ha cambiado: gira mas y repite."));
  } else if (d > 0) {
    Serial.println(F("  El valor SUBE en sentido horario: es lo que el"));
    Serial.println(F("  datasheet da con DIR a GND."));
  } else {
    Serial.println(F("  El valor BAJA en sentido horario: o DIR esta a VCC,"));
    Serial.println(F("  o al aire, o lo has mirado desde el otro lado."));
  }
  Serial.println(F("  En el robot los cuatro sensores tienen que llevar DIR"));
  Serial.println(F("  igual (a GND). El espejado de FL y RR lo hace el software."));
  Serial.println();
}

void ayuda() {
  Serial.println();
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe la letra y pulsa Enter)          |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  s   Estado: iman, ganancia, angulo, memoria        |"));
  Serial.println(F("|  c   Lectura continua (Enter para parar)            |"));
  Serial.println(F("|  r   Ruido con el eje quieto                        |"));
  Serial.println(F("|  v   Una vuelta completa: iman centrado?            |"));
  Serial.println(F("|  d   Sentido de giro                                |"));
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
  Serial.println(F("  StarCrawler - ENCODERS, PASO 2 de 5: UN AS5600"));
  Serial.println(F("=================================================="));
  Serial.println(F("Un encoder directo al ESP32, sin multiplexor."));
  Serial.println(F("Solo lee: no escribe nada en el chip."));
  estado();
  ayuda();
}

void loop() {
  if (!Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  switch (linea.charAt(0)) {
    case 's': case 'S': estado();   break;
    case 'c': case 'C': continuo(); break;
    case 'r': case 'R': ruido();    break;
    case 'v': case 'V': vuelta();   break;
    case 'd': case 'D': sentido();  break;
    case 'h': case 'H': case '?': ayuda(); break;
    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
  }
}
