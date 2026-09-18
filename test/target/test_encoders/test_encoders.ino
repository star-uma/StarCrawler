/*
 * test_encoders.ino - StarCrawler - PRUEBA 1 de 4
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   Los 4 sensores de angulo (AS5600) que miden cuanto esta girada
 *   cada oruga, y el multiplexor (TCA9548A), que es el chip que
 *   permite leer los 4 sensores con un unico bus I2C.
 *
 *   RIESGO: NINGUNO. Aqui no se mueve ningun motor.
 *   Es la primera prueba que hay que hacer y la mas segura.
 *
 *   QUE NECESITAS CONECTADO
 *     - ESP32 DevKit V1 conectado por USB al PC
 *     - Multiplexor TCA9548A en el bus I2C (pin 21 = SDA, 22 = SCL)
 *     - Los 4 encoders AS5600 en los canales 0, 1, 2 y 3 del mux
 *     - NO hace falta alimentar los motores
 *
 *   COMO SE USA
 *     1. Compila y sube el sketch (ver test/target/README.md)
 *     2. Abre el Monitor Serie a 115200 baudios
 *     3. Escribe 'h' y pulsa Enter para ver los comandos
 *
 *   QUE TIENES QUE CONSEGUIR (apuntalo en algun sitio)
 *     a) Que los 4 encoders respondan            -> comando 's'
 *     b) Saber que canal es que oruga de verdad  -> comando 'c',
 *        moviendo cada brazo A MANO y mirando cual cambia
 *     c) Los offsets de calibracion              -> comando 'o',
 *        con las 4 orugas puestas en horizontal
 *
 * ================================================================
 */

#include <Wire.h>

/* ---- CONFIGURACION --------------------------------------------
 * Estos valores salen de firmware/starcrawler_esp32/config.h.
 * Si alli cambian, hay que cambiarlos aqui tambien.
 */
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define DIR_TCA9548A  0x70   /* direccion I2C del multiplexor */
#define DIR_AS5600    0x36   /* direccion I2C de cada encoder  */
#define NUM_ORUGAS    4

/* Orden usado en TODO el proyecto: {FR, FL, RR, RL}
 *   FR = Front Right  (delantera derecha)
 *   FL = Front Left   (delantera izquierda)
 *   RR = Rear Right   (trasera derecha)
 *   RL = Rear Left    (trasera izquierda)
 */
const char* NOMBRE[NUM_ORUGAS] = {"FR", "FL", "RR", "RL"};

/* Offsets que tiene ahora el firmware. Son del robot de 2025 y hay
 * que recalibrarlos: justamente para eso esta el comando 'o'. */
float offsets[NUM_ORUGAS] = { -2.0f, -5.0f, 16.0f, -15.0f };

/* ---- ESTADO INTERNO -------------------------------------------- */
bool  modoContinuo = false;
int   canalUnico   = -1;          /* -1 = mostrar las 4 orugas */
float ultimoAngulo[NUM_ORUGAS];
bool  hayUltimo[NUM_ORUGAS]  = {false, false, false, false};
int   vecesIgual[NUM_ORUGAS] = {0, 0, 0, 0};

/* ---- FUNCIONES DE BAJO NIVEL ----------------------------------- */

/* Le dice al multiplexor "abre el canal N". Todo lo que hablemos
 * por I2C despues de esto ira al encoder de ese canal. */
bool abrirCanal(uint8_t canal) {
  if (canal > 7) return false;
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

/* Lee el angulo crudo del AS5600: 0..4095 = una vuelta completa.
 * Devuelve false si el encoder no contesta. */
bool leerCrudo(int idx, uint16_t *crudo) {
  if (!abrirCanal(idx)) return false;

  /* Pedimos el registro 0x0C (RAW ANGLE). Los dos bytes se leen de
   * una sola vez para que no se mezclen dos muestras distintas. */
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_AS5600, 2) != 2) return false;

  uint16_t alto = Wire.read();
  uint16_t bajo = Wire.read();
  *crudo = (uint16_t)(((alto & 0x0F) << 8) | bajo);
  return true;
}

/* Convierte el valor crudo a grados y le suma el offset de montaje.
 * Es exactamente la misma formula que usa el firmware. */
float aGrados(uint16_t crudo, float offset) {
  return (float)crudo * 0.087890625f + offset;   /* 360 / 4096 */
}

/* Espera a que el usuario escriba algo. Devuelve lo escrito en
 * mayusculas, o cadena vacia si pasa el tiempo limite. */
String esperarRespuesta(unsigned long milisegundos) {
  unsigned long limite = millis() + milisegundos;
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

/* ---- COMANDOS -------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe la letra y pulsa Enter)          |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  s   Escanear: ver que responde en el bus I2C       |"));
  Serial.println(F("|  c   Lectura continua de las 4 orugas.              |"));
  Serial.println(F("|      Usalo moviendo cada brazo A MANO.              |"));
  Serial.println(F("|      Pulsa Enter para parar.                        |"));
  Serial.println(F("|  1   Solo oruga FR       3   Solo oruga RR          |"));
  Serial.println(F("|  2   Solo oruga FL       4   Solo oruga RL          |"));
  Serial.println(F("|  o   Asistente de offsets (orugas EN HORIZONTAL)    |"));
  Serial.println(F("|  h   Mostrar esta ayuda                             |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println();
}

void escanearBus() {
  Serial.println();
  Serial.println(F("=== ESCANEO DEL BUS I2C ==="));

  /* 1) El multiplexor */
  Wire.beginTransmission(DIR_TCA9548A);
  if (Wire.endTransmission() == 0) {
    Serial.println(F("[OK]    Multiplexor TCA9548A encontrado en 0x70"));
  } else {
    Serial.println(F("[ERROR] NO se encuentra el multiplexor TCA9548A (0x70)"));
    Serial.println(F("        Revisa:"));
    Serial.println(F("          - que este alimentado a 3.3 V"));
    Serial.println(F("          - los cables SDA (pin 21) y SCL (pin 22)"));
    Serial.println(F("          - que A0, A1 y A2 esten conectados a GND"));
    Serial.println(F("        Sin el mux no se puede leer ningun encoder."));
    Serial.println();
    return;
  }

  /* 2) Un encoder por canal */
  int encontrados = 0;
  for (int i = 0; i < NUM_ORUGAS; i++) {
    uint16_t crudo;
    if (leerCrudo(i, &crudo)) {
      Serial.print(F("[OK]    Canal "));
      Serial.print(i);
      Serial.print(F(" responde   (deberia ser la oruga "));
      Serial.print(NOMBRE[i]);
      Serial.print(F(", crudo = "));
      Serial.print(crudo);
      Serial.println(F(")"));
      encontrados++;
    } else {
      Serial.print(F("[ERROR] Canal "));
      Serial.print(i);
      Serial.print(F(" NO responde   (deberia ser la oruga "));
      Serial.print(NOMBRE[i]);
      Serial.println(F(")"));
    }
  }

  Serial.println();
  Serial.print(F("Resultado: "));
  Serial.print(encontrados);
  Serial.println(F(" de 4 encoders responden."));
  if (encontrados == 4) {
    Serial.println(F(">>> Bien. Siguiente paso: comando 'c'."));
  } else {
    Serial.println(F(">>> Arregla el cableado de los que fallan ANTES de seguir."));
  }
  Serial.println();
}

void cabeceraContinuo() {
  Serial.println();
  if (canalUnico >= 0) {
    Serial.print(F("=== LECTURA CONTINUA: solo la oruga "));
    Serial.print(NOMBRE[canalUnico]);
    Serial.println(F(" ==="));
  } else {
    Serial.println(F("=== LECTURA CONTINUA: las 4 orugas ==="));
  }
  Serial.println();
  Serial.println(F("Mueve cada brazo A MANO y mira cual columna cambia."));
  Serial.println(F("Asi confirmas que canal corresponde a que oruga."));
  Serial.println(F("Pulsa Enter para parar."));
  Serial.println();
  Serial.println(F("       crudo    grados   con offset   estado"));
}

void lineaContinuo() {
  for (int i = 0; i < NUM_ORUGAS; i++) {
    if (canalUnico >= 0 && i != canalUnico) continue;

    uint16_t crudo;
    Serial.print(NOMBRE[i]);
    Serial.print(F("   "));

    if (!leerCrudo(i, &crudo)) {
      Serial.println(F("  ----     ----       ----      NO RESPONDE"));
      hayUltimo[i] = false;
      continue;
    }

    float sinOffset = aGrados(crudo, 0.0f);
    float conOffset = aGrados(crudo, offsets[i]);

    /* Relleno para que las columnas queden alineadas */
    if (crudo < 1000) Serial.print(F(" "));
    if (crudo < 100)  Serial.print(F(" "));
    if (crudo < 10)   Serial.print(F(" "));
    Serial.print(F("  "));
    Serial.print(crudo);
    Serial.print(F("    "));
    Serial.print(sinOffset, 1);
    Serial.print(F("      "));
    Serial.print(conOffset, 1);
    Serial.print(F("      "));

    /* Avisos utiles: lectura congelada o salto brusco */
    if (hayUltimo[i]) {
      float salto = fabsf(conOffset - ultimoAngulo[i]);
      if (salto < 0.05f) vecesIgual[i]++;
      else               vecesIgual[i] = 0;

      if (salto > 20.0f) {
        Serial.print(F("SALTO BRUSCO de "));
        Serial.print(salto, 1);
        Serial.print(F(" grados <- sospechoso"));
      } else if (vecesIgual[i] > 40) {
        Serial.print(F("sin cambios (normal si no lo mueves)"));
      } else {
        Serial.print(F("ok"));
      }
    } else {
      Serial.print(F("ok"));
    }
    Serial.println();
  }
  Serial.println();

  /* Guardamos las lecturas para comparar en la siguiente vuelta */
  for (int i = 0; i < NUM_ORUGAS; i++) {
    if (canalUnico >= 0 && i != canalUnico) continue;
    uint16_t crudo;
    if (leerCrudo(i, &crudo)) {
      ultimoAngulo[i] = aGrados(crudo, offsets[i]);
      hayUltimo[i] = true;
    }
  }
}

void asistenteOffsets() {
  Serial.println();
  Serial.println(F("=== ASISTENTE DE OFFSETS ==="));
  Serial.println();
  Serial.println(F("PARA QUE SIRVE"));
  Serial.println(F("El firmware necesita saber que valor marca cada encoder"));
  Serial.println(F("cuando su oruga esta HORIZONTAL. Por convencion del"));
  Serial.println(F("proyecto, horizontal = 180 grados."));
  Serial.println();
  Serial.println(F("ANTES DE CONTINUAR:"));
  Serial.println(F("  1. Pon las CUATRO orugas lo mas horizontales posible."));
  Serial.println(F("  2. Usa un nivel o una regla si tienes."));
  Serial.println(F("  3. No las toques durante la medida (tarda ~2 s)."));
  Serial.println();
  Serial.println(F("Escribe SI y pulsa Enter cuando esten colocadas."));
  Serial.println(F("(cualquier otra cosa cancela)"));

  String respuesta = esperarRespuesta(120000UL);
  if (respuesta != "SI") {
    Serial.println(F("Cancelado. No se ha medido nada."));
    Serial.println();
    return;
  }

  Serial.println();
  Serial.println(F("Midiendo (50 muestras por oruga)..."));

  float nuevos[NUM_ORUGAS];
  bool  ok[NUM_ORUGAS];

  for (int i = 0; i < NUM_ORUGAS; i++) {
    float suma = 0.0f;
    int   validas = 0;
    for (int m = 0; m < 50; m++) {
      uint16_t crudo;
      if (leerCrudo(i, &crudo)) {
        suma += aGrados(crudo, 0.0f);
        validas++;
      }
      delay(10);
    }
    if (validas >= 25) {
      float medio = suma / (float)validas;
      nuevos[i] = 180.0f - medio;   /* offset que lleva la lectura a 180 */
      ok[i] = true;
    } else {
      ok[i] = false;
    }
  }

  Serial.println();
  Serial.println(F("=== RESULTADO ==="));
  Serial.println();
  bool todoOk = true;
  for (int i = 0; i < NUM_ORUGAS; i++) {
    Serial.print(F("  "));
    Serial.print(NOMBRE[i]);
    Serial.print(F(" : "));
    if (ok[i]) {
      Serial.print(F("offset nuevo = "));
      Serial.print(nuevos[i], 1);
      Serial.print(F("      (el firmware tiene ahora "));
      Serial.print(offsets[i], 1);
      Serial.println(F(")"));
    } else {
      Serial.println(F("FALLO: el encoder no respondio lo suficiente"));
      todoOk = false;
    }
  }

  Serial.println();
  if (todoOk) {
    Serial.println(F("Copia esta linea en config.h, sustituyendo la que hay:"));
    Serial.println();
    Serial.print(F("#define OFFSETS_ENCODER { "));
    for (int i = 0; i < NUM_ORUGAS; i++) {
      Serial.print(nuevos[i], 1);
      Serial.print(F("f"));
      if (i < NUM_ORUGAS - 1) Serial.print(F(", "));
    }
    Serial.println(F(" }"));
    Serial.println();
    Serial.println(F("OJO: hay que cambiarla en las 4 variantes de firmware"));
    Serial.println(F("(esp32, esp32_basico, esp32_standalone, esp32_ros2)."));
  } else {
    Serial.println(F("No se genera la linea porque fallo algun encoder."));
  }
  Serial.println();
}

/* ---- ARRANQUE -------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(1500);                    /* tiempo para abrir el monitor serie */

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - PRUEBA 1 de 4: ENCODERS"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("Esta prueba NO mueve ningun motor. Es segura."));
  Serial.println(F("Sirve para medir el angulo de las 4 orugas."));
  ayuda();

  escanearBus();
}

void loop() {
  /* Modo de lectura continua */
  if (modoContinuo) {
    lineaContinuo();
    delay(250);
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      modoContinuo = false;
      canalUnico = -1;
      Serial.println(F(">>> Parado. Escribe 'h' para ver los comandos."));
      Serial.println();
    }
    return;
  }

  /* Menu */
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  char c = linea.charAt(0);

  switch (c) {
    case 's': case 'S':
      escanearBus();
      break;

    case 'c': case 'C':
      canalUnico = -1;
      for (int i = 0; i < NUM_ORUGAS; i++) { hayUltimo[i] = false; vecesIgual[i] = 0; }
      cabeceraContinuo();
      modoContinuo = true;
      break;

    case '1': case '2': case '3': case '4':
      canalUnico = c - '1';
      for (int i = 0; i < NUM_ORUGAS; i++) { hayUltimo[i] = false; vecesIgual[i] = 0; }
      cabeceraContinuo();
      modoContinuo = true;
      break;

    case 'o': case 'O':
      asistenteOffsets();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.print(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      Serial.println();
      break;
  }
}
