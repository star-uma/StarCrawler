/*
 * test_i2c_escaner.ino - StarCrawler - ENCODERS, PASO 1 de 5
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   El bus I2C en si: los dos cables (SDA y SCL) que comparten los
 *   encoders, el multiplexor y la IMU. Antes de culpar a un sensor,
 *   hay que saber si el bus esta sano:
 *     - que las lineas tienen resistencias de pull-up
 *     - que ninguna esta cortocircuitada a GND
 *     - que responde y en que direccion
 *
 *   RIESGO: NINGUNO. No mueve nada ni escribe en ningun sensor.
 *
 *   QUE NECESITAS CONECTADO
 *   Lo que tengas en el bus, aunque sea nada. Se puede usar en
 *   cualquier momento del montaje: con el ESP32 solo, con un
 *   encoder, con el multiplexor, con todo.
 *
 *     ESP32        bus I2C
 *     3V3   --->   VCC de cada modulo
 *     GND   --->   GND de cada modulo
 *     GPIO21 -->   SDA
 *     GPIO22 -->   SCL
 *
 *   COMO SE USA
 *     1. Compila y sube el sketch (ver test/target/README.md)
 *     2. Monitor Serie a 115200 baudios
 *     3. Al arrancar hace todas las comprobaciones. 'h' = ayuda
 *
 *   QUE ESPERAR CON EL ROBOT COMPLETO
 *     0x70  multiplexor TCA9548A
 *     0x68  IMU MPU9250 (o 0x69 si su pin AD0 esta en alto)
 *     y NINGUN 0x36 en el bus principal: los encoders van detras
 *     del multiplexor y solo aparecen al abrir su canal ('m').
 *
 * ================================================================
 */

#include <Wire.h>

/* ---- CONFIGURACION --------------------------------------------
 * Los mismos pines que el firmware (hw_comun.h en feature/ros2).
 */
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define DIR_AS5600    0x36

/* ---- ESTADO ---------------------------------------------------- */

int  dirMux = -1;                 /* -1 = no hay multiplexor */

/* ---- NOMBRES DE LO QUE SE PUEDE ENCONTRAR ---------------------- */

const char* nombreDispositivo(uint8_t dir) {
  if (dir == 0x36)                return "encoder AS5600";
  if (dir >= 0x70 && dir <= 0x77) return "multiplexor TCA9548A";
  if (dir == 0x68 || dir == 0x69) return "IMU MPU9250 / MPU6050";
  if (dir == 0x0C)                return "magnetometro AK8963 (dentro de la MPU9250)";
  return "desconocido";
}

/* ---- I2C ------------------------------------------------------- */

void arrancarI2C(uint32_t frecuencia) {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(frecuencia);
  Wire.setTimeOut(20);            /* ms: que un bus colgado no bloquee */
}

bool responde(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

/* Cierra todos los canales del multiplexor, si lo hay. Asi lo que
 * responda en el bus principal esta de verdad en el bus principal. */
void cerrarMux() {
  if (dirMux < 0) return;
  Wire.beginTransmission((uint8_t)dirMux);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
}

/* ---- 1. LAS LINEAS --------------------------------------------- */

/* Lee una linea con el pull-down interno y luego con el pull-up
 * interno. Con eso se distingue:
 *   - pull-up externo bueno : se lee 1 incluso con el pull-down
 *   - sin pull-up externo   : 0 con pull-down, 1 con pull-up (flota)
 *   - linea pegada a GND    : 0 incluso con el pull-up
 */
void comprobarLinea(const char* nombre, int pin, bool *ok) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(5);
  int conBajada = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);
  delay(5);
  int conSubida = digitalRead(pin);
  pinMode(pin, INPUT);

  Serial.print(F("  "));
  Serial.print(nombre);
  Serial.print(F(" (GPIO"));
  Serial.print(pin);
  Serial.print(F(")  "));

  if (conBajada == 1) {
    Serial.println(F("[OK]    tiene pull-up"));
  } else if (conSubida == 1) {
    Serial.println(F("[AVISO] SIN pull-up (o muy debil): la linea flota"));
    Serial.println(F("          El I2C necesita una resistencia de 4,7 kOhm de"));
    Serial.println(F("          la linea a 3,3 V. Los modulos suelen traerla;"));
    Serial.println(F("          si no hay ningun modulo conectado, es normal."));
    *ok = false;
  } else {
    Serial.println(F("[ERROR] PEGADA A GND: algo la tira a 0"));
    Serial.println(F("          O hay un corto a GND, o un dispositivo se ha"));
    Serial.println(F("          quedado a medias en una transaccion."));
    Serial.println(F("          Prueba 'r' (recuperar el bus); si sigue, revisa"));
    Serial.println(F("          el cableado de esa linea modulo a modulo."));
    *ok = false;
  }
}

void comprobarLineas() {
  Serial.println();
  Serial.println(F("=== 1. LAS LINEAS SDA Y SCL ==="));
  Wire.end();                     /* soltar los pines para leerlos a mano */
  bool ok = true;
  comprobarLinea("SDA", PIN_I2C_SDA, &ok);
  comprobarLinea("SCL", PIN_I2C_SCL, &ok);
  arrancarI2C(100000);
  if (ok) Serial.println(F(">>> Las lineas estan bien."));
  Serial.println();
}

/* Si un esclavo se quedo a mitad de un byte (por un reset del ESP32,
 * por ejemplo), tiene SDA en bajo esperando relojes. Nueve pulsos de
 * SCL lo sacan de ahi. */
void recuperarBus() {
  Serial.println();
  Serial.println(F("=== RECUPERAR EL BUS: 9 pulsos de reloj ==="));
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  delay(2);
  Serial.print(F("  SDA despues de los pulsos: "));
  Serial.println(digitalRead(PIN_I2C_SDA) ? F("alta (bien)") : F("SIGUE BAJA: corto o cableado"));
  pinMode(PIN_I2C_SDA, INPUT);
  pinMode(PIN_I2C_SCL, INPUT);
  arrancarI2C(100000);
  Serial.println();
}

/* ---- 2. QUE RESPONDE ------------------------------------------- */

/* Busca el multiplexor en sus 8 direcciones posibles. */
void buscarMux() {
  dirMux = -1;
  for (uint8_t d = 0x70; d <= 0x77; d++) {
    if (responde(d)) { dirMux = d; return; }
  }
}

/* Escanea 0x08..0x77 (las direcciones normales del I2C).
 * Devuelve cuantos dispositivos hay y los marca en 'visto'. */
int escanear(bool visto[128]) {
  int n = 0;
  for (uint8_t d = 0x08; d <= 0x77; d++) {
    visto[d] = responde(d);
    if (visto[d]) n++;
  }
  return n;
}

void escanearPrincipal() {
  Serial.println();
  Serial.println(F("=== 2. QUE RESPONDE EN EL BUS PRINCIPAL ==="));

  arrancarI2C(100000);
  buscarMux();
  cerrarMux();

  bool a100[128] = {false};
  bool a400[128] = {false};
  int n = escanear(a100);
  Wire.setClock(400000);
  escanear(a400);
  Wire.setClock(100000);

  if (n == 0) {
    Serial.println(F("  Nada responde."));
    Serial.println(F("  Si hay algo conectado: alimentacion (3,3 V y GND),"));
    Serial.println(F("  SDA y SCL cruzados, o falta el pull-up (paso 1)."));
    Serial.println();
    return;
  }

  bool diferencias = false;
  for (uint8_t d = 0x08; d <= 0x77; d++) {
    if (!a100[d] && !a400[d]) continue;
    Serial.print(F("  0x"));
    if (d < 0x10) Serial.print(F("0"));
    Serial.print(d, HEX);
    Serial.print(F("  "));
    Serial.print(nombreDispositivo(d));
    if (a100[d] && !a400[d]) {
      Serial.print(F("   <- responde a 100 kHz pero NO a 400 kHz"));
      diferencias = true;
    }
    Serial.println();
  }
  Serial.println();

  /* Interpretacion para este robot */
  if (dirMux == 0x70) {
    Serial.println(F("  [OK]    Multiplexor en 0x70, la direccion del firmware."));
  } else if (dirMux > 0x70) {
    Serial.print(F("  [ERROR] Multiplexor en 0x"));
    Serial.print(dirMux, HEX);
    Serial.println(F(", y el firmware lo espera en 0x70."));
    Serial.println(F("          Sus pines A0, A1 y A2 tienen que ir a GND."));
  }
  if (a100[DIR_AS5600]) {
    if (dirMux >= 0) {
      Serial.println(F("  [AVISO] Hay un AS5600 en el bus PRINCIPAL con el mux"));
      Serial.println(F("          cerrado: va conectado directo a SDA/SCL y no a"));
      Serial.println(F("          un canal. En el robot eso no puede ser: los cuatro"));
      Serial.println(F("          tienen la misma direccion y se pisarian."));
    } else {
      Serial.println(F("  [OK]    Un AS5600 directo al bus: es el montaje del"));
      Serial.println(F("          paso 2 (test_as5600_solo)."));
    }
  }
  if (diferencias) {
    Serial.println(F("  [AVISO] Algo falla a 400 kHz, que es a lo que va el"));
    Serial.println(F("          firmware. Suele ser cable largo o pull-ups flojas."));
  }
  if (dirMux >= 0) {
    Serial.println();
    Serial.println(F(">>> Hay multiplexor: 'm' mira detras de cada canal."));
  }
  Serial.println();
}

/* ---- 3. DETRAS DE CADA CANAL DEL MUX --------------------------- */

void escanearCanales() {
  Serial.println();
  Serial.println(F("=== 3. DETRAS DE CADA CANAL DEL MULTIPLEXOR ==="));
  arrancarI2C(100000);
  buscarMux();
  if (dirMux < 0) {
    Serial.println(F("  No hay multiplexor. Para probarlo: test_tca9548a."));
    Serial.println();
    return;
  }
  cerrarMux();
  bool base[128] = {false};
  escanear(base);

  const char* oruga[8] = {"FR", "FL", "RR", "RL", "-", "-", "-", "-"};
  for (uint8_t c = 0; c < 8; c++) {
    Wire.beginTransmission((uint8_t)dirMux);
    Wire.write((uint8_t)(1 << c));
    if (Wire.endTransmission() != 0) {
      Serial.println(F("  [ERROR] El multiplexor no acepta la orden de canal."));
      break;
    }
    bool aqui[128] = {false};
    escanear(aqui);

    Serial.print(F("  canal "));
    Serial.print(c);
    Serial.print(F(" ("));
    Serial.print(oruga[c]);
    Serial.print(F("):"));
    int n = 0;
    for (uint8_t d = 0x08; d <= 0x77; d++) {
      if (!aqui[d] || base[d]) continue;     /* solo lo nuevo de este canal */
      Serial.print(F("  0x"));
      Serial.print(d, HEX);
      Serial.print(F(" "));
      Serial.print(nombreDispositivo(d));
      n++;
    }
    if (n == 0) Serial.print(F("  (nada)"));
    Serial.println();
  }
  cerrarMux();
  Serial.println();
  Serial.println(F("  En el robot: un AS5600 en los canales 0 a 3 y nada en"));
  Serial.println(F("  los demas. Mas detalle por canal: test_tca9548a."));
  Serial.println();
}

/* ---- MENU ------------------------------------------------------ */

void ayuda() {
  Serial.println();
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe la letra y pulsa Enter)          |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println(F("|  l   Comprobar las lineas (pull-ups y cortos)       |"));
  Serial.println(F("|  s   Escanear el bus principal                      |"));
  Serial.println(F("|  m   Escanear detras de cada canal del mux          |"));
  Serial.println(F("|  r   Recuperar un bus colgado (SDA pegada a GND)    |"));
  Serial.println(F("|  t   Todo: l + s + m                                |"));
  Serial.println(F("|  h   Esta ayuda                                     |"));
  Serial.println(F("+-----------------------------------------------------+"));
  Serial.println();
}

void todo() {
  comprobarLineas();
  escanearPrincipal();
  if (dirMux >= 0) escanearCanales();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  arrancarI2C(100000);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - ENCODERS, PASO 1 de 5: BUS I2C"));
  Serial.println(F("=================================================="));
  Serial.println(F("No mueve nada ni escribe en ningun sensor."));
  todo();
  ayuda();
}

void loop() {
  if (!Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  switch (linea.charAt(0)) {
    case 'l': case 'L': comprobarLineas();   break;
    case 's': case 'S': escanearPrincipal(); break;
    case 'm': case 'M': escanearCanales();   break;
    case 'r': case 'R': recuperarBus();      break;
    case 't': case 'T': todo();              break;
    case 'h': case 'H': case '?': ayuda();   break;
    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
  }
}
