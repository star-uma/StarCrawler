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
 *     PUL+ / DIR+ / ENA+ del DM542 -> +3.3V  (NO a 5V, ver abajo)
 *
 *   *** OJO CON LOS 3.3V ***
 *   Las entradas del DM542 son optoacopladores. Si los pines "+" van a
 *   5V y el ESP32 (3.3V) pone la senal en ALTO, quedan 1.7V sobre el
 *   opto: no se apaga, y el driver ve la senal siempre activada.
 *   Con DIR eso significa que el motor gira SIEMPRE HACIA EL MISMO LADO,
 *   aunque el sketch diga que cambia de sentido. Con los "+" a 3.3V el
 *   nivel ALTO deja 0V sobre el opto y apaga de verdad.
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
 * 2000 us -> unos 250 pasos por segundo. Con la reductora 1:80 eso son
 * ~2.8 grados/s en el brazo: se mueve despacio y hay que mirar con
 * calma para verlo. */
#define SEMIPERIODO_STEP_US  2000

/* Rampa de aceleracion (mismos valores que test_steppers_all).
 * Arrancar de golpe a la velocidad de regimen es la causa tipica de que
 * el motor pierda pasos con la inercia del brazo. Con 'r' se activa y
 * desactiva para comparar el mismo motor con y sin ella. */
#define SEMIPERIODO_ARRANQUE_US  4800
#define RAMPA_DECREMENTO_US       100

/* En el DM542: ENA en ALTO = driver deshabilitado (motor suelto).
 *              ENA en BAJO = driver habilitado (motor con par). */
#define ENA_HABILITADO  LOW
#define ENA_APAGADO     HIGH

/* ---- ESTADO ------------------------------------------------------ */
bool armado    = false;   /* hasta que no se arma, no se mueve nada */
int  numPasos  = 20000;     /* pasos por movimiento */
bool conRampa  = true;      /* rampa de aceleracion (comando 'r') */

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
  unsigned int semi = conRampa ? SEMIPERIODO_ARRANQUE_US : SEMIPERIODO_STEP_US;
  for (int p = 0; p < pasos; p++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(semi);
    digitalWrite(PIN_STEP, LOW);
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

  apagarMotor();

  Serial.println();
  if (cortado) Serial.println(F("  Cortado por el usuario."));
  Serial.println(F("  Movimiento terminado."));
  Serial.println();
}

/* ---- COMANDOS -------------------------------------------------- */

/* Comprueba el pin DIR SIN mover el motor. Es la prueba decisiva cuando
 * el motor gira siempre hacia el mismo lado pase lo que pase. */
void probarDir() {
  apagarMotor();          /* driver deshabilitado: no puede girar nada */

  Serial.println();
  Serial.println(F("=== PRUEBA DEL PIN DIR (no mueve el motor) ==="));
  Serial.println();
  Serial.println(F("Pon el polimetro entre DIR+ y DIR- del DM542 y ve"));
  Serial.println(F("anotando lo que marca en cada paso."));

  for (int i = 0; i < 2; i++) {
    const bool alto = (i == 0);
    digitalWrite(PIN_DIR, alto ? HIGH : LOW);
    Serial.println();
    Serial.print(F("  DIR en "));
    Serial.print(alto ? F("ALTO") : F("BAJO"));
    Serial.println(F("  -> mide ahora. Pulsa Enter para seguir."));
    leerLinea(120000UL);
  }

  digitalWrite(PIN_DIR, LOW);
  Serial.println();
  Serial.println(F("=== COMO INTERPRETARLO ==="));
  Serial.println();
  Serial.println(F("  BAJO ~3.3 V  y  ALTO ~0 V"));
  Serial.println(F("    Correcto. Los pines + estan a 3.3 V y el driver ve"));
  Serial.println(F("    el cambio de sentido."));
  Serial.println();
  Serial.println(F("  BAJO ~5 V  y  ALTO ~1.7 V"));
  Serial.println(F("    *** ESTE ES EL FALLO ***  Los pines + estan a 5 V."));
  Serial.println(F("    Esos 1.7 V no apagan el optoacoplador, asi que el"));
  Serial.println(F("    driver ve DIR siempre activado y el motor gira"));
  Serial.println(F("    SIEMPRE HACIA EL MISMO LADO."));
  Serial.println(F("    Solucion: pasar PUL+, DIR+ y ENA+ de 5 V a 3.3 V."));
  Serial.println();
}

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
  Serial.println(F("|  r   Activar/desactivar la RAMPA de aceleracion      |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  d   PROBAR EL PIN DIR (no mueve el motor)           |"));
  Serial.println(F("|      Usalo si el motor gira siempre hacia el mismo   |"));
  Serial.println(F("|      lado con + y con -                              |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  h   Mostrar esta ayuda                              |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.print(F("  Pasos por movimiento: "));
  Serial.print(numPasos);
  Serial.print(F("    Rampa: "));
  Serial.println(conRampa ? F("SI") : F("NO"));
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

    case 'r': case 'R':
      conRampa = !conRampa;
      Serial.print(F(">>> Rampa de aceleracion: "));
      Serial.println(conRampa ? F("ACTIVADA") : F("DESACTIVADA"));
      Serial.println();
      break;

    case 'd': case 'D':
      probarDir();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      break;
  }
}