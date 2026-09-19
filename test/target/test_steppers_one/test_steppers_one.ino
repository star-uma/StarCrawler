/*
 * test_stepper_solo.ino - StarCrawler - prueba de UN solo motor
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   Un unico motor paso a paso (stepper) con su driver DM542,
 *   sin encoders ni I2C. Solo sirve para comprobar que el motor
 *   gira y que el sentido del pin DIR es el que esperas.
 *
 *   *** ESTA PRUEBA MUEVE UN MOTOR ***
 *   *** ASEGURATE DE QUE EL BRAZO PUEDE MOVERSE SIN CHOCAR ***
 *
 *   CABLEADO (motor FR, segun config.h)
 *     STEP (PUL-) -> GPIO 25
 *     DIR  (DIR-) -> GPIO 33
 *     ENA  (ENA-) -> GPIO 4
 *     PUL+ / DIR+ / ENA+ del DM542 -> +5V comun con GND del ESP32
 *
 *   COMO SE USA
 *     1. Compila y sube este sketch
 *     2. Monitor Serie a 115200 baudios
 *     3. Escribe 'h' para ver los comandos
 *     4. Para poder mover algo hay que "armar" primero: comando 'a'
 *
 * ================================================================
 */

/* ---- CONFIGURACION --------------------------------------------- */
#define PIN_STEP   25
#define PIN_DIR    33
#define PIN_ENA    4

/* Medio periodo del pulso STEP, en microsegundos.
 * 1200 us -> unos 417 pasos por segundo. */
#define SEMIPERIODO_STEP_US  2000

/* En el DM542: ENA en ALTO = driver deshabilitado (motor suelto).
 *              ENA en BAJO = driver habilitado (motor con par). */
#define ENA_HABILITADO  LOW
#define ENA_APAGADO     HIGH

/* ---- ESTADO ------------------------------------------------------ */
bool armado    = false;   /* hasta que no se arma, no se mueve nada */
int  numPasos  = 20000;     /* pasos por movimiento */

/* ---- MOTOR --------------------------------------------------------- */

void apagarMotor() {
  digitalWrite(PIN_ENA, ENA_APAGADO);
  digitalWrite(PIN_STEP, LOW);
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

/* Mueve el motor el numero de pasos indicado.
 * sentido = +1 -> DIR en HIGH
 * sentido = -1 -> DIR en LOW
 * Se puede cortar pulsando Enter durante el movimiento. */
void moverMotor(int sentido, int pasos) {
  Serial.println();
  Serial.print(F("Moviendo motor  ("));
  Serial.print(pasos);
  Serial.print(F(" pasos, sentido "));
  Serial.print(sentido > 0 ? F("+") : F("-"));
  Serial.println(F(")"));
  Serial.println(F("  >>> MIRA EL MOTOR: gira? en que sentido?"));
  Serial.println(F("  (pulsa Enter para cortar)"));
  delay(800);

  digitalWrite(PIN_DIR, sentido > 0 ? HIGH : LOW);
  delayMicroseconds(50);          /* el DM542 pide un margen */
  digitalWrite(PIN_ENA, ENA_HABILITADO);
  delay(10);

  bool cortado = false;
  for (int p = 0; p < pasos; p++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(SEMIPERIODO_STEP_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(SEMIPERIODO_STEP_US);

    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      cortado = true;
      break;
    }
  }

  apagarMotor();

  Serial.println();
  if (cortado) Serial.println(F("  Cortado por el usuario."));
  Serial.println(F("  Movimiento terminado."));
  Serial.println();
}

/* ---- COMANDOS -------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe y pulsa Enter)                    |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  a   ARMAR: habilita el movimiento (pide confirmar)  |"));
  Serial.println(F("|  p   PARAR: apaga el driver y desarma                |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  +   Mover el motor en sentido +                     |"));
  Serial.println(F("|  -   Mover el motor en sentido -                     |"));
  Serial.println(F("|  n   Cambiar cuantos pasos se dan por movimiento     |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  h   Mostrar esta ayuda                              |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.print(F("  Pasos por movimiento: "));
  Serial.println(numPasos);
  Serial.print(F("  Estado: "));
  Serial.println(armado ? F("ARMADO (puede mover)") : F("desarmado (no mueve)"));
  Serial.println();
}

void armar() {
  Serial.println();
  Serial.println(F("=== ARMAR EL SISTEMA ==="));
  Serial.println();
  Serial.println(F("*** A PARTIR DE AQUI EL MOTOR SE PUEDE MOVER ***"));
  Serial.println();
  Serial.println(F("Comprueba:"));
  Serial.println(F("  [ ] El brazo/motor tiene sitio para moverse sin chocar"));
  Serial.println(F("  [ ] Nadie tiene las manos cerca"));
  Serial.println(F("  [ ] Sabes donde esta el interruptor de los motores"));
  Serial.println();
  Serial.println(F("Escribe SI y pulsa Enter para armar."));
  Serial.println(F("(cualquier otra cosa lo deja desarmado)"));

  String r = leerLinea(120000UL);
  if (r == "SI") {
    armado = true;
    Serial.println();
    Serial.println(F(">>> ARMADO. Muevelo con + o -."));
    Serial.println(F(">>> Con 'p' paras y desarmas en cualquier momento."));
  } else {
    armado = false;
    Serial.println(F("Sigue desarmado. No se movera nada."));
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

  /* Lo PRIMERO: dejar el driver apagado, antes de nada mas.
   * Asi el motor no puede moverse durante el arranque. */
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, ENA_APAGADO);
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);

/* Comunicación I2C con encoder
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); */

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - PRUEBA DE UN SOLO MOTOR"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("*** ESTA PRUEBA MUEVE UN MOTOR ***"));
  Serial.println();
  Serial.println(F("El driver arranca APAGADO. No se movera nada"));
  Serial.println(F("hasta que armes el sistema con el comando 'a'."));

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
      apagarMotor();
      armado = false;
      Serial.println();
      Serial.println(F("*** PARADO: driver apagado y desarmado ***"));
      Serial.println();
      break;

    case '+':
      if (!armado) {
        Serial.println(F("Esta desarmado. Usa 'a' para armar primero."));
        Serial.println();
      } else {
        moverMotor(+1, numPasos);
      }
      break;

      /* Deberia moverse para atrás. Probar conexiones DIR y alimentación*/
    case '-':
      if (!armado) {
        Serial.println(F("Esta desarmado. Usa 'a' para armar primero."));
        Serial.println();
      } else {
        moverMotor(-1, numPasos);
      }
      break;

    case 'n': case 'N':
      cambiarPasos();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      break;
  }
}