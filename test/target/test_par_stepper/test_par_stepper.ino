/*
 * test_par_stepper.ino - StarCrawler - por que un stepper pierde pasos
 * ================================================================
 *
 *   PARA QUE SIRVE
 *   Diagnosticar por que un motor de elevacion pierde pasos y "gruñe"
 *   cuando se le mete algo de carga. Pensado para UN SOLO motor y SIN
 *   encoder: solo hace falta tu mano y una marca de referencia.
 *
 *   *** ESTA PRUEBA MUEVE UN MOTOR ***
 *   *** ROBOT SOBRE TACOS / BRAZO LIBRE ***
 *
 *   QUE NECESITAS
 *     - ESP32 DevKit V1 por USB
 *     - UN driver DM542 cableado (el del motor que quieras probar)
 *     - El motor alimentado
 *     - NO hace falta encoder ni multiplexor
 *
 *   LA IDEA
 *   Hay tres causas tipicas y cada prueba descarta una:
 *
 *     'r' PAR DE RETENCION - el motor se queda quieto y energizado, sin
 *         pulsos. Si con la mano puedes moverlo, NO es un problema de
 *         dinamica: es corriente o cableado. Es la prueba que mas
 *         informacion da y la primera que hay que hacer.
 *
 *     'v' BARRIDO DE VELOCIDAD - lo mueve a varias velocidades. Si hay
 *         una velocidad donde va flojo y otras donde va fuerte, es
 *         RESONANCIA (se cura con mas micropasos). Si va flojo a todas,
 *         es corriente o cableado.
 *
 *     'i' IDA Y VUELTA - da N pasos y los deshace. Sin pasos perdidos
 *         vuelve exactamente a la marca. Lo que se desvie son pasos
 *         perdidos. Repetible sin encoder.
 *
 * ================================================================
 */

#include <Arduino.h>

/* ---- CONFIGURACION --------------------------------------------
 * Valores de firmware/starcrawler_esp32/config.h. Orden {FR,FL,RR,RL}
 */
#define NUM_MOTORES 4
const char* NOMBRE[NUM_MOTORES] = {"FR", "FL", "RR", "RL"};

const int PIN_STEP[NUM_MOTORES] = { 25, 26, 27, 32 };
const int PIN_DIR [NUM_MOTORES] = { 33, 13, 14, 15 };
const int PIN_ENA [NUM_MOTORES] = {  4, 16, 17,  2 };

/* En el DM542: ENA alto = driver apagado (motor suelto) */
#define ENA_HABILITADO  LOW
#define ENA_APAGADO     HIGH

/* Velocidad de regimen del firmware: 1200 us de semiperiodo = 417 pasos/s */
#define SEMIPERIODO_REGIMEN_US 1200

/* ---- ESTADO ---------------------------------------------------- */
bool armado = false;
int  motorElegido = 0;
int  numPasos = 400;      /* 400 pasos = 1 vuelta del motor a 400 pasos/vuelta */

/* ---- UTILIDADES ------------------------------------------------ */

/* delayMicroseconds pierde precision por encima de ~16 ms: partimos. */
void esperaUs(uint32_t us) {
  while (us > 10000) { delay(10); us -= 10000; }
  if (us) delayMicroseconds(us);
}

void apagarTodos() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    digitalWrite(PIN_ENA[i], ENA_APAGADO);
    digitalWrite(PIN_STEP[i], LOW);
  }
}

String leerLinea(unsigned long msLimite) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    if (Serial.available()) {
      String r = Serial.readStringUntil('\n');
      r.trim(); r.toUpperCase();
      return r;
    }
    delay(10);
  }
  return "";
}

bool comprobarArmado() {
  if (!armado) {
    Serial.println(F("Primero hay que ARMAR: comando 'a'."));
    return false;
  }
  return true;
}

/* Genera 'pasos' pulsos al motor con el semiperiodo indicado.
 * Devuelve true si el usuario corto pulsando Enter. */
bool pulsar(int motor, bool sentido, long pasos, uint32_t semiUs) {
  digitalWrite(PIN_DIR[motor], sentido ? HIGH : LOW);
  delayMicroseconds(20);                 /* margen DIR->STEP del DM542 */
  digitalWrite(PIN_ENA[motor], ENA_HABILITADO);
  delay(5);

  for (long p = 0; p < pasos; p++) {
    digitalWrite(PIN_STEP[motor], HIGH);
    esperaUs(semiUs);
    digitalWrite(PIN_STEP[motor], LOW);
    esperaUs(semiUs);
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      apagarTodos();
      Serial.println(F("  >>> cortado."));
      return true;
    }
  }
  return false;
}

/* ---- PRUEBAS --------------------------------------------------- */

void parDeRetencion() {
  if (!comprobarArmado()) return;

  Serial.println();
  Serial.println(F("=== PRUEBA 1: PAR DE RETENCION ==="));
  Serial.println();
  Serial.println(F("El motor se queda QUIETO y energizado, sin pulsos."));
  Serial.println(F("Empuja el brazo con la mano e intenta moverlo."));
  Serial.println();
  Serial.println(F("COMO INTERPRETARLO"));
  Serial.println(F("  - Si NO puedes moverlo (o te cuesta mucho): el motor"));
  Serial.println(F("    tiene par. El problema es dinamico -> prueba 'v'."));
  Serial.println(F("  - Si lo mueves con facilidad: NO le llega corriente."));
  Serial.println(F("    Causas: DIP de corriente bajos, una fase suelta o"));
  Serial.println(F("    mal emparejada, o la fuente no da. Mira la ayuda 'd'."));
  Serial.println();
  Serial.print(F("Energizando "));
  Serial.print(NOMBRE[motorElegido]);
  Serial.println(F(" durante 15 s (Enter para cortar antes)..."));

  apagarTodos();
  digitalWrite(PIN_ENA[motorElegido], ENA_HABILITADO);

  unsigned long fin = millis() + 15000;
  while (millis() < fin) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    delay(50);
  }
  apagarTodos();
  Serial.println(F("Listo. Motor suelto."));
  Serial.println();
  Serial.println(F(">>> APUNTA: se podia mover con la mano?  SI / NO"));
  Serial.println();
}

void barridoVelocidad() {
  if (!comprobarArmado()) return;

  Serial.println();
  Serial.println(F("=== PRUEBA 2: BARRIDO DE VELOCIDAD ==="));
  Serial.println();
  Serial.println(F("Va a girar a 7 velocidades, 4 s cada una."));
  Serial.println(F("EN CADA UNA: empuja con la mano y fijate si aguanta."));
  Serial.println(F("Escribe SI para empezar."));
  if (leerLinea(60000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  /* semiperiodo en us y pasos/s equivalentes */
  const uint32_t semis[7] = {20000, 10000, 5000, 2500, 1200, 625, 312};
  const int      pps[7]   = {   25,    50,  100,  200,  417, 800, 1602};

  for (int k = 0; k < 7; k++) {
    Serial.println();
    Serial.print(F("--- "));
    Serial.print(pps[k]);
    Serial.print(F(" pasos/s  (semiperiodo "));
    Serial.print(semis[k]);
    Serial.print(F(" us)"));
    if (semis[k] == SEMIPERIODO_REGIMEN_US) {
      Serial.print(F("  <-- la del firmware"));
    }
    Serial.println(F(" ---"));
    Serial.println(F("    EMPUJA AHORA"));

    long pasos = (long)(4000000.0 / (2.0 * (double)semis[k]));  /* 4 segundos */
    if (pasos < 1) pasos = 1;
    if (pulsar(motorElegido, true, pasos, semis[k])) return;
    apagarTodos();
    delay(700);
  }

  Serial.println();
  Serial.println(F("FIN DEL BARRIDO."));
  Serial.println();
  Serial.println(F("COMO INTERPRETARLO"));
  Serial.println(F("  - Fuerte a unas velocidades y flojo a otras (y gruñe"));
  Serial.println(F("    justo en las flojas): es RESONANCIA. Solucion: subir"));
  Serial.println(F("    micropasos en los DIP (SW5-SW8) de 400 a 1600, y"));
  Serial.println(F("    ajustar SEMIPERIODO_STEP_US en config.h para mantener"));
  Serial.println(F("    la misma velocidad de brazo."));
  Serial.println(F("  - Flojo a TODAS: es corriente o cableado. Ayuda 'd'."));
  Serial.println();
}

void idaYVuelta() {
  if (!comprobarArmado()) return;

  Serial.println();
  Serial.println(F("=== PRUEBA 3: IDA Y VUELTA (pasos perdidos) ==="));
  Serial.println();
  Serial.println(F("PREPARACION: pon una marca en el brazo y otra enfrente"));
  Serial.println(F("en el chasis, alineadas (cinta, rotulador...)."));
  Serial.println();
  Serial.print(F("Dara "));
  Serial.print(numPasos);
  Serial.println(F(" pasos en un sentido y los mismos de vuelta, 5 veces."));
  Serial.println(F("Sin pasos perdidos, vuelve SIEMPRE a la marca."));
  Serial.println(F("Lo que se desvie son pasos que se han perdido."));
  Serial.println();
  Serial.println(F("Escribe SI para empezar."));
  if (leerLinea(60000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  for (int ciclo = 1; ciclo <= 5; ciclo++) {
    Serial.print(F("  ciclo "));
    Serial.print(ciclo);
    Serial.println(F(" de 5..."));
    if (pulsar(motorElegido, true,  numPasos, SEMIPERIODO_REGIMEN_US)) return;
    delay(400);
    if (pulsar(motorElegido, false, numPasos, SEMIPERIODO_REGIMEN_US)) return;
    apagarTodos();
    delay(800);
    Serial.println(F("     -> mira la marca. Ha vuelto a su sitio?"));
  }

  Serial.println();
  Serial.println(F("FIN. Si la marca se ha ido desplazando, hay pasos"));
  Serial.println(F("perdidos. Si vuelve clavada, el motor no pierde nada"));
  Serial.println(F("a esta velocidad y sin carga."));
  Serial.println();
}

void ayudaDips() {
  Serial.println();
  Serial.println(F("=== DIP DEL DM542 (comprobar con el driver APAGADO) ==="));
  Serial.println();
  Serial.println(F("CORRIENTE (SW1 SW2 SW3)   pico / reposo"));
  Serial.println(F("  ON  ON  ON   1.00 A / 0.71     <- el mas flojo"));
  Serial.println(F("  OFF ON  ON   1.46 A / 1.04"));
  Serial.println(F("  ON  OFF ON   1.91 A / 1.36"));
  Serial.println(F("  OFF OFF ON   2.37 A / 1.69"));
  Serial.println(F("  ON  ON  OFF  2.84 A / 2.03"));
  Serial.println(F("  OFF ON  OFF  3.31 A / 2.36"));
  Serial.println(F("  ON  OFF OFF  3.76 A / 2.69"));
  Serial.println(F("  OFF OFF OFF  4.20 A / 3.00     <- el maximo"));
  Serial.println();
  Serial.println(F("El motor 57HS112 es de 4.2 A: los tres en OFF."));
  Serial.println(F("Si estan en otra posicion, estas regalando par."));
  Serial.println();
  Serial.println(F("SW4 - corriente en reposo"));
  Serial.println(F("  OFF = la mitad en reposo (por defecto)"));
  Serial.println(F("  ON  = la misma en reposo (mas par de retencion,"));
  Serial.println(F("        mas calor)"));
  Serial.println();
  Serial.println(F("MICROPASOS (SW5-SW8): ahora deberia estar en 400/vuelta."));
  Serial.println(F("Subir a 1600 reduce muchisimo la resonancia."));
  Serial.println();
  Serial.println(F("SI EL PAR DE RETENCION ES FLOJO, comprueba ademas:"));
  Serial.println(F("  - Emparejado de fases: los 4 hilos del motor van en"));
  Serial.println(F("    DOS pares (A+/A- y B+/B-). Si se mezclan los pares,"));
  Serial.println(F("    el motor vibra, gruñe y casi no tiene par. Con el"));
  Serial.println(F("    motor desconectado, mide continuidad: los dos hilos"));
  Serial.println(F("    de un mismo par dan unos pocos ohmios entre si, y"));
  Serial.println(F("    circuito abierto contra los del otro par."));
  Serial.println(F("  - Tension: el DM542 admite 20-50 V. A mas tension,"));
  Serial.println(F("    mas par en movimiento."));
  Serial.println(F("  - Que la fuente de un la corriente (4.2 A por motor)."));
  Serial.println();
}

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  a   ARMAR (pide confirmar)                          |"));
  Serial.println(F("|  p   PARAR: apaga los drivers y desarma              |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  1..4  Elegir motor (1=FR 2=FL 3=RR 4=RL)            |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  r   PRUEBA 1: par de retencion  <- EMPIEZA POR AQUI |"));
  Serial.println(F("|  v   PRUEBA 2: barrido de velocidad                  |"));
  Serial.println(F("|  i   PRUEBA 3: ida y vuelta (pasos perdidos)         |"));
  Serial.println(F("|  n   Cambiar pasos de la prueba 3                    |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  d   Tabla de DIP del DM542 y que comprobar          |"));
  Serial.println(F("|  h   Esta ayuda                                      |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.print(F("  Motor: "));
  Serial.print(NOMBRE[motorElegido]);
  Serial.print(F("    Pasos prueba 3: "));
  Serial.print(numPasos);
  Serial.print(F("    Estado: "));
  Serial.println(armado ? F("ARMADO") : F("desarmado"));
  Serial.println();
}

/* ---- ARRANQUE -------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(400);

  for (int i = 0; i < NUM_MOTORES; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    pinMode(PIN_ENA[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_DIR[i], LOW);
    digitalWrite(PIN_ENA[i], ENA_APAGADO);
  }

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - por que el stepper pierde pasos"));
  Serial.println(F("  (un solo motor, sin encoder)"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("ROBOT SOBRE TACOS antes de mover nada."));
  ayuda();
  Serial.println(F("Empieza por 'a' (armar) y luego 'r' (par de retencion)."));
  Serial.println();
}

void loop() {
  if (!Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  char c = linea.charAt(0);

  switch (c) {
    case 'a': case 'A':
      Serial.println();
      Serial.println(F("Vas a ARMAR: a partir de ahora los comandos mueven"));
      Serial.println(F("el motor. El robot esta sobre tacos?"));
      Serial.println(F("Escribe SI para confirmar."));
      if (leerLinea(60000UL) == "SI") {
        armado = true;
        Serial.println(F(">>> ARMADO."));
      } else {
        Serial.println(F("Cancelado, sigue desarmado."));
      }
      Serial.println();
      break;

    case 'p': case 'P':
      apagarTodos();
      armado = false;
      Serial.println(F(">>> Drivers apagados y desarmado."));
      break;

    case '1': case '2': case '3': case '4':
      motorElegido = c - '1';
      Serial.print(F("Motor elegido: "));
      Serial.println(NOMBRE[motorElegido]);
      break;

    case 'r': case 'R': parDeRetencion(); break;
    case 'v': case 'V': barridoVelocidad(); break;
    case 'i': case 'I': idaYVuelta(); break;

    case 'n': case 'N': {
      Serial.println(F("Cuantos pasos por sentido? (100-20000)"));
      String s = leerLinea(30000UL);
      int n = s.toInt();
      if (n >= 100 && n <= 20000) {
        numPasos = n;
        Serial.print(F("Pasos: "));
        Serial.println(numPasos);
      } else {
        Serial.println(F("Valor no valido, se queda como estaba."));
      }
      break;
    }

    case 'd': case 'D': ayudaDips(); break;
    case 'h': case 'H': case '?': ayuda(); break;
    default: Serial.println(F("No entiendo. Escribe 'h'.")); break;
  }
}
