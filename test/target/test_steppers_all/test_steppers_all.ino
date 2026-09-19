/*
 * test_steppers.ino - StarCrawler - PRUEBA 3 de 4
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   Los 4 motores paso a paso (steppers) que levantan y bajan las
 *   orugas, con sus drivers DM542. Se mueven DE UNO EN UNO.
 *
 *   *** ESTA PRUEBA MUEVE MOTORES ***
 *   *** EL ROBOT TIENE QUE ESTAR SOBRE TACOS ***
 *
 *   QUE NECESITAS CONECTADO
 *     - ESP32 DevKit V1 por USB al PC
 *     - Los 4 drivers DM542 cableados segun config.h
 *     - Los motores paso a paso alimentados
 *     - Los encoders (prueba 1) conectados: se usan para ver si el
 *       brazo se mueve de verdad y hacia donde
 *
 *   HAZ ANTES LA PRUEBA 1 (test_encoders)
 *   Necesitas saber que canal es que oruga antes de venir aqui.
 *
 *   QUE TIENES QUE CONSEGUIR
 *     a) Que cada motor mueva el brazo que le toca (y solo ese)
 *     b) Saber que sentido del pin DIR hace que el brazo SUBA
 *     c) Confirmar que el angulo del encoder cambia como toca
 *
 *   COMO SE USA
 *     1. Compila y sube (ver test/target/README.md)
 *     2. Monitor Serie a 115200 baudios
 *     3. Escribe 'h' para ver los comandos
 *     4. Para poder mover algo hay que "armar" primero: comando 'a'
 *
 * ================================================================
 */

#include <Wire.h>

/* ---- CONFIGURACION --------------------------------------------
 * Valores sacados de firmware/starcrawler_esp32/config.h.
 * Si alli cambian, cambialos aqui tambien.
 */
#define NUM_ORUGAS   4

/* Orden en todos los vectores: {FR, FL, RR, RL} */
const char* NOMBRE[NUM_ORUGAS] = {"FR", "FL", "RR", "RL"};

const int PIN_STEP[NUM_ORUGAS] = { 25, 26, 27, 32 };
const int PIN_DIR [NUM_ORUGAS] = { 33, 13, 14, 15 };
const int PIN_ENA [NUM_ORUGAS] = {  4, 16, 17,  2 };

/* Nivel del pin DIR para que el angulo del encoder AUMENTE.
 * Esto es lo que hay que verificar en esta prueba. */
const int DIR_ANGULO_SUBE[NUM_ORUGAS] = { 0, 1, 1, 0 };

/* Medio periodo del pulso STEP, en microsegundos.
 * 1200 us -> unos 417 pasos por segundo. */
#define SEMIPERIODO_STEP_US  1200

/* Rampa de aceleracion (mismos valores que firmware/.../config.h).
 * Arrancar de golpe a la velocidad de regimen es la causa tipica de que el
 * motor pierda pasos con una carga inercial como el brazo de la oruga.
 * Con el comando 'r' se puede activar y desactivar para comparar. */
#define SEMIPERIODO_ARRANQUE_US  4800
#define RAMPA_DECREMENTO_US       100

/* En el DM542: ENA en ALTO = driver deshabilitado (motor suelto).
 *              ENA en BAJO = driver habilitado (motor con par). */
#define ENA_HABILITADO  LOW
#define ENA_APAGADO     HIGH

/* Encoders, para ver si el brazo se mueve de verdad */
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define DIR_TCA9548A  0x70
#define DIR_AS5600    0x36

float offsets[NUM_ORUGAS] = { -2.0f, -5.0f, 16.0f, -15.0f };

/* ---- ESTADO ---------------------------------------------------- */
bool armado       = false;   /* hasta que no se arma, no se mueve nada */
int  motorElegido = 0;       /* 0=FR 1=FL 2=RR 3=RL */
int  numPasos     = 200;     /* pasos por movimiento */
bool conRampa     = true;    /* rampa de aceleracion (comando 'r') */

/* ---- ENCODERS (igual que en la prueba 1) ----------------------- */

bool abrirCanal(uint8_t canal) {
  if (canal > 7) return false;
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

bool leerAngulo(int idx, float *grados) {
  if (!abrirCanal(idx)) return false;
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_AS5600, 2) != 2) return false;
  uint16_t alto = Wire.read();
  uint16_t bajo = Wire.read();
  uint16_t crudo = (uint16_t)(((alto & 0x0F) << 8) | bajo);
  *grados = (float)crudo * 0.087890625f + offsets[idx];
  return true;
}

/* ---- MOTORES --------------------------------------------------- */

/* Deja los 4 drivers apagados (motores sueltos). */
void apagarTodos() {
  for (int i = 0; i < NUM_ORUGAS; i++) {
    digitalWrite(PIN_ENA[i], ENA_APAGADO);
    digitalWrite(PIN_STEP[i], LOW);
  }
}

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

/* Mueve UN motor el numero de pasos indicado.
 * sentido = +1 -> el angulo del encoder deberia AUMENTAR
 * sentido = -1 -> deberia DISMINUIR
 * Se puede cortar pulsando Enter durante el movimiento. */
void moverMotor(int motor, int sentido, int pasos) {
  float angInicial = 0.0f;
  bool  hayInicial = leerAngulo(motor, &angInicial);

  Serial.println();
  Serial.print(F("Moviendo "));
  Serial.print(NOMBRE[motor]);
  Serial.print(F("  ("));
  Serial.print(pasos);
  Serial.print(F(" pasos, sentido "));
  Serial.print(sentido > 0 ? F("+") : F("-"));
  Serial.print(F(", rampa "));
  Serial.print(conRampa ? F("SI") : F("NO"));
  Serial.println(F(")"));

  if (hayInicial) {
    Serial.print(F("  angulo antes : "));
    Serial.println(angInicial, 1);
  } else {
    Serial.println(F("  angulo antes : (el encoder no responde)"));
  }

  Serial.println(F("  >>> MIRA EL ROBOT: que brazo se mueve?"));
  Serial.println(F("  (pulsa Enter para cortar)"));
  delay(800);

  /* Solo se habilita el driver de ESTE motor. Los otros tres siguen
   * apagados, asi no puede moverse nada mas por error. */
  apagarTodos();
  int nivelDir = (sentido > 0) ? DIR_ANGULO_SUBE[motor]
                               : !DIR_ANGULO_SUBE[motor];
  digitalWrite(PIN_DIR[motor], nivelDir ? HIGH : LOW);
  delayMicroseconds(50);                       /* el DM542 pide un margen */
  digitalWrite(PIN_ENA[motor], ENA_HABILITADO);
  delay(10);

  bool cortado = false;
  unsigned int semi = conRampa ? SEMIPERIODO_ARRANQUE_US : SEMIPERIODO_STEP_US;
  for (int p = 0; p < pasos; p++) {
    digitalWrite(PIN_STEP[motor], HIGH);
    delayMicroseconds(semi);
    digitalWrite(PIN_STEP[motor], LOW);
    delayMicroseconds(semi);

    /* acelerar: acortar el semiperiodo hasta el de regimen */
    if (semi > SEMIPERIODO_STEP_US) {
      unsigned int margen = semi - SEMIPERIODO_STEP_US;
      semi -= (margen < RAMPA_DECREMENTO_US) ? margen : RAMPA_DECREMENTO_US;
    }

    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      cortado = true;
      break;
    }
  }

  apagarTodos();
  delay(200);   /* que el brazo se asiente antes de volver a medir */

  float angFinal = 0.0f;
  bool  hayFinal = leerAngulo(motor, &angFinal);

  Serial.println();
  if (cortado) Serial.println(F("  Cortado por el usuario."));
  Serial.print(F("  angulo despues: "));
  if (hayFinal) Serial.println(angFinal, 1);
  else          Serial.println(F("(el encoder no responde)"));

  if (hayInicial && hayFinal) {
    float delta = angFinal - angInicial;
    Serial.print(F("  ha cambiado   : "));
    Serial.print(delta, 1);
    Serial.println(F(" grados"));
    Serial.println();

    if (fabsf(delta) < 0.5f) {
      Serial.println(F("  *** EL ANGULO NO HA CAMBIADO ***"));
      Serial.println(F("  Posibles causas:"));
      Serial.println(F("    - El driver no esta alimentado"));
      Serial.println(F("    - El cableado STEP/DIR/ENA no es el de config.h"));
      Serial.println(F("    - El motor esta desacoplado del brazo"));
      Serial.println(F("    - Este encoder no es el de este brazo"));
      Serial.println(F("      (repite la prueba 1 para confirmarlo)"));
    } else {
      bool subioAngulo = (delta > 0.0f);
      bool esperado    = (sentido > 0);
      if (subioAngulo == esperado) {
        Serial.println(F("  OK: el angulo se ha movido en el sentido esperado."));
        Serial.println(F("  La tabla DIR de config.h es correcta para este motor."));
      } else {
        Serial.println(F("  *** SENTIDO AL REVES ***"));
        Serial.print(F("  Para este motor ("));
        Serial.print(NOMBRE[motor]);
        Serial.println(F(") hay que invertir su valor en"));
        Serial.println(F("  TABLA_DIR_HORARIO / TABLA_DIR_ANTIHORARIO de config.h,"));
        Serial.println(F("  o cambiar el cable DIR."));
      }
    }
  }

  Serial.println();
  Serial.println(F("  APUNTA: que brazo se ha movido y hacia donde"));
  Serial.println(F("  (subiendo = se levanta del suelo)"));
  Serial.println();
}

/* ---- COMANDOS -------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe y pulsa Enter)                    |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  a   ARMAR: habilita el movimiento (pide confirmar)  |"));
  Serial.println(F("|  p   PARAR: apaga los 4 drivers y desarma            |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  1   Elegir motor FR     3   Elegir motor RR         |"));
  Serial.println(F("|  2   Elegir motor FL     4   Elegir motor RL         |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  +   Mover el motor elegido (angulo deberia SUBIR)   |"));
  Serial.println(F("|  -   Mover el motor elegido (angulo deberia BAJAR)   |"));
  Serial.println(F("|  n   Cambiar cuantos pasos se dan por movimiento     |"));
  Serial.println(F("|  r   Activar/desactivar la RAMPA de aceleracion      |"));
  Serial.println(F("|      (sin rampa = como antes: arranque de golpe)     |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  e   Ver el angulo de las 4 orugas ahora mismo       |"));
  Serial.println(F("|  h   Mostrar esta ayuda                              |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.print(F("  Motor elegido ahora: "));
  Serial.print(NOMBRE[motorElegido]);
  Serial.print(F("    Pasos por movimiento: "));
  Serial.println(numPasos);
  Serial.print(F("  Estado: "));
  Serial.println(armado ? F("ARMADO (puede mover)") : F("desarmado (no mueve)"));
  Serial.println();
}

void armar() {
  Serial.println();
  Serial.println(F("=== ARMAR EL SISTEMA ==="));
  Serial.println();
  Serial.println(F("*** A PARTIR DE AQUI LOS MOTORES SE PUEDEN MOVER ***"));
  Serial.println();
  Serial.println(F("Comprueba:"));
  Serial.println(F("  [ ] El robot esta SOBRE TACOS, con las orugas al aire"));
  Serial.println(F("  [ ] Nadie tiene las manos cerca de los brazos"));
  Serial.println(F("  [ ] Sabes donde esta el interruptor de los motores"));
  Serial.println(F("  [ ] Los brazos tienen sitio para moverse sin chocar"));
  Serial.println();
  Serial.println(F("Escribe SI y pulsa Enter para armar."));
  Serial.println(F("(cualquier otra cosa lo deja desarmado)"));

  String r = leerLinea(120000UL);
  if (r == "SI") {
    armado = true;
    Serial.println();
    Serial.println(F(">>> ARMADO. Elige motor (1-4) y muevelo con + o -."));
    Serial.println(F(">>> Con 'p' paras y desarmas en cualquier momento."));
  } else {
    armado = false;
    Serial.println(F("Sigue desarmado. No se movera nada."));
  }
  Serial.println();
}

void mostrarAngulos() {
  Serial.println();
  Serial.println(F("=== ANGULOS AHORA MISMO ==="));
  for (int i = 0; i < NUM_ORUGAS; i++) {
    float a;
    Serial.print(F("  "));
    Serial.print(NOMBRE[i]);
    Serial.print(F(" : "));
    if (leerAngulo(i, &a)) Serial.println(a, 1);
    else                   Serial.println(F("NO RESPONDE"));
  }
  Serial.println();
}

void cambiarPasos() {
  Serial.println();
  Serial.print(F("Pasos por movimiento ahora: "));
  Serial.println(numPasos);
  Serial.println(F("Escribe el numero nuevo (entre 10 y 2000) y pulsa Enter."));
  Serial.println(F("Referencia: 200 pasos son unos 0.5 segundos de giro."));

  String r = leerLinea(60000UL);
  int v = r.toInt();
  if (v >= 10 && v <= 2000) {
    numPasos = v;
    Serial.print(F(">>> Ahora se daran "));
    Serial.print(numPasos);
    Serial.println(F(" pasos por movimiento."));
  } else {
    Serial.println(F("Valor no valido. Se queda como estaba."));
  }
  Serial.println();
}

/* ---- ARRANQUE -------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(1500);

  /* Lo PRIMERO: dejar los drivers apagados, antes de nada mas.
   * Asi el robot no puede moverse durante el arranque. */
  for (int i = 0; i < NUM_ORUGAS; i++) {
    pinMode(PIN_ENA[i], OUTPUT);
    digitalWrite(PIN_ENA[i], ENA_APAGADO);
    pinMode(PIN_STEP[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_DIR[i], LOW);
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - PRUEBA 3 de 4: STEPPERS"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("*** ESTA PRUEBA MUEVE MOTORES ***"));
  Serial.println(F("*** EL ROBOT TIENE QUE ESTAR SOBRE TACOS ***"));
  Serial.println();
  Serial.println(F("Los 4 drivers arrancan APAGADOS. No se movera nada"));
  Serial.println(F("hasta que armes el sistema con el comando 'a'."));
  Serial.println();
  Serial.println(F("Haz antes la prueba 1 (test_encoders): necesitas saber"));
  Serial.println(F("que encoder corresponde a que brazo."));

  mostrarAngulos();
  ayuda();
}

void loop() {
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  char c = linea.charAt(0);

  switch (c) {
    case 'a': case 'A':
      armar();
      break;

    case 'p': case 'P':
      apagarTodos();
      armado = false;
      Serial.println();
      Serial.println(F("*** PARADO: los 4 drivers apagados y desarmado ***"));
      Serial.println();
      break;

    case '1': case '2': case '3': case '4':
      motorElegido = c - '1';
      Serial.print(F(">>> Motor elegido: "));
      Serial.println(NOMBRE[motorElegido]);
      Serial.println();
      break;

    case '+':
      if (!armado) {
        Serial.println(F("Esta desarmado. Usa 'a' para armar primero."));
        Serial.println();
      } else {
        moverMotor(motorElegido, +1, numPasos);
      }
      break;

    case '-':
      if (!armado) {
        Serial.println(F("Esta desarmado. Usa 'a' para armar primero."));
        Serial.println();
      } else {
        moverMotor(motorElegido, -1, numPasos);
      }
      break;

    case 'n': case 'N':
      cambiarPasos();
      break;

    case 'r': case 'R':
      conRampa = !conRampa;
      Serial.println();
      Serial.print(F(">>> Rampa de aceleracion: "));
      Serial.println(conRampa ? F("ACTIVADA") : F("DESACTIVADA"));
      if (conRampa) {
        Serial.println(F("    Arranca a 4800 us de semiperiodo y acelera"));
        Serial.println(F("    hasta 1200 us en unos 36 pasos."));
      } else {
        Serial.println(F("    Arranque de golpe a 1200 us, como antes."));
      }
      Serial.println();
      break;

    case 'e': case 'E':
      mostrarAngulos();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      break;
  }
}