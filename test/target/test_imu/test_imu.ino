/*
 * test_imu.ino - StarCrawler - PRUEBA 4 de 4
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   La IMU (MPU9250): el sensor que dice como esta inclinado el
 *   robot. Da dos angulos:
 *     - roll  (alabeo): inclinacion hacia los lados
 *     - pitch (cabeceo): inclinacion hacia delante o atras
 *
 *   RIESGO: NINGUNO. Aqui no se mueve ningun motor.
 *
 *   POR QUE ES IMPORTANTE
 *   El modo 5 del robot (nivelado automatico) usa estos dos angulos
 *   para corregir la postura sola. Si un signo esta al reves, el
 *   robot corregira JUSTO AL CONTRARIO y se volcara.
 *   Por eso esta prueba hay que hacerla SI O SI antes de usar el
 *   modo 5 con el robot en el suelo.
 *
 *   QUE NECESITAS CONECTADO
 *     - ESP32 DevKit V1 por USB al PC
 *     - MPU9250 en el bus I2C del ESP32 (21 = SDA, 22 = SCL)
 *       OJO: en el robot del TFG la IMU estaba en el Arduino MKR.
 *       Para la arquitectura nueva hay que RECABLEARLA al ESP32.
 *     - NO hace falta alimentar los motores
 *
 *   QUE TIENES QUE CONSEGUIR
 *     a) Que la IMU responda                      -> comando 's'
 *     b) Que los signos sean los correctos        -> comando 'v'
 *        (es el objetivo principal de esta prueba)
 *
 *   CONVENIO DEL PROYECTO (figura 7.40 del TFG)
 *     roll  POSITIVO = robot inclinado hacia la DERECHA
 *     pitch POSITIVO = robot inclinado hacia ATRAS
 *
 * ================================================================
 */

#include <Wire.h>

/* ---- CONFIGURACION --------------------------------------------
 * Valores sacados de firmware/starcrawler_esp32/config.h
 */
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22
#define DIR_MPU9250  0x68

/* Signos que tiene ahora el firmware. Esta prueba sirve para
 * comprobar si son correctos o hay que darles la vuelta. */
#define SIGNO_ROLL   (+1.0f)
#define SIGNO_PITCH  (+1.0f)

#define RAD_A_DEG  57.29577951f

/* ---- ESTADO ---------------------------------------------------- */
bool  imuOk        = false;
bool  modoContinuo = false;
float sesgoGiro[3] = {0.0f, 0.0f, 0.0f};

/* ---- BAJO NIVEL ------------------------------------------------ */

bool escribirReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(DIR_MPU9250);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

/* Lee acelerometro y giroscopio de golpe (14 bytes desde 0x3B). */
bool leerCrudo(int16_t acc[3], int16_t gyr[3]) {
  Wire.beginTransmission(DIR_MPU9250);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_MPU9250, 14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  acc[0] = (int16_t)((b[0]  << 8) | b[1]);
  acc[1] = (int16_t)((b[2]  << 8) | b[3]);
  acc[2] = (int16_t)((b[4]  << 8) | b[5]);
  /* b[6] y b[7] son la temperatura, no la usamos */
  gyr[0] = (int16_t)((b[8]  << 8) | b[9]);
  gyr[1] = (int16_t)((b[10] << 8) | b[11]);
  gyr[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

/* Calcula roll y pitch a partir de la gravedad medida por el
 * acelerometro. Es la misma formula que usa el firmware. */
bool leerAngulos(float *roll, float *pitch) {
  int16_t acc[3], gyr[3];
  if (!leerCrudo(acc, gyr)) return false;

  const float ax = (float)acc[0] / 16384.0f;   /* escala +-2 g */
  const float ay = (float)acc[1] / 16384.0f;
  const float az = (float)acc[2] / 16384.0f;

  *pitch = SIGNO_PITCH * atan2f(az, ax) * RAD_A_DEG;
  *roll  = SIGNO_ROLL  * atan2f(ay, ax) * RAD_A_DEG;
  return true;
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

/* ---- COMANDOS -------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  COMANDOS (escribe la letra y pulsa Enter)           |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  s   Buscar la IMU en el bus I2C                     |"));
  Serial.println(F("|  c   Ver roll y pitch continuamente (Enter para      |"));
  Serial.println(F("|      parar). Inclina el robot y mira como cambian.   |"));
  Serial.println(F("|  v   VERIFICAR SIGNOS (guiado paso a paso)           |"));
  Serial.println(F("|      <- este es el objetivo de la prueba             |"));
  Serial.println(F("|  g   Calibrar la deriva del giroscopo                |"));
  Serial.println(F("|  r   Ver valores crudos (para diagnostico)           |"));
  Serial.println(F("|  h   Mostrar esta ayuda                              |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println();
}

void buscarIMU() {
  Serial.println();
  Serial.println(F("=== BUSCANDO LA IMU ==="));

  Wire.beginTransmission(DIR_MPU9250);
  if (Wire.endTransmission() != 0) {
    imuOk = false;
    Serial.println(F("[ERROR] La IMU NO responde en la direccion 0x68."));
    Serial.println();
    Serial.println(F("Revisa:"));
    Serial.println(F("  - Que este cableada al I2C del ESP32 (21 y 22)."));
    Serial.println(F("    En el robot del TFG estaba en el Arduino MKR:"));
    Serial.println(F("    para la arquitectura nueva hay que recablearla."));
    Serial.println(F("  - Alimentacion a 3.3 V y GND comun"));
    Serial.println(F("  - Si el pin AD0 esta a VCC, la direccion es 0x69"));
    Serial.println(F("    en vez de 0x68: cambia DIR_MPU9250 arriba."));
    Serial.println();
    return;
  }

  Serial.println(F("[OK]    La IMU responde en 0x68"));
  Serial.print(F("        Configurando... "));

  bool ok = true;
  ok &= escribirReg(0x6B, 0x00);   /* salir del modo reposo */
  delay(50);
  ok &= escribirReg(0x1B, 0x00);   /* giroscopo +-250 grados/s */
  ok &= escribirReg(0x1C, 0x00);   /* acelerometro +-2 g */
  ok &= escribirReg(0x1A, 0x03);   /* filtro paso bajo 41 Hz */

  imuOk = ok;
  Serial.println(ok ? F("OK") : F("FALLO al configurar"));

  if (imuOk) {
    float roll, pitch;
    if (leerAngulos(&roll, &pitch)) {
      Serial.println();
      Serial.print(F("        Lectura actual -> roll "));
      Serial.print(roll, 1);
      Serial.print(F("   pitch "));
      Serial.println(pitch, 1);
      Serial.println();
      Serial.println(F(">>> Siguiente paso: comando 'v' para verificar signos."));
    }
  }
  Serial.println();
}

void calibrarGiro() {
  if (!imuOk) {
    Serial.println(F("La IMU no esta lista. Usa 's' primero."));
    return;
  }
  Serial.println();
  Serial.println(F("=== CALIBRAR DERIVA DEL GIROSCOPO ==="));
  Serial.println(F("NO MUEVAS EL ROBOT durante unos 2 segundos..."));

  float suma[3] = {0, 0, 0};
  int validas = 0;
  for (int i = 0; i < 500; i++) {
    int16_t acc[3], gyr[3];
    if (leerCrudo(acc, gyr)) {
      for (int e = 0; e < 3; e++) suma[e] += (float)gyr[e];
      validas++;
    }
    delay(2);
  }
  if (validas > 0) {
    for (int e = 0; e < 3; e++) sesgoGiro[e] = suma[e] / (float)validas;
    Serial.println(F("Listo. Deriva medida (en cuentas):"));
    Serial.print(F("  X "));  Serial.print(sesgoGiro[0], 1);
    Serial.print(F("   Y ")); Serial.print(sesgoGiro[1], 1);
    Serial.print(F("   Z ")); Serial.println(sesgoGiro[2], 1);
    Serial.println(F("Valores muy grandes (>200) indican que la IMU se"));
    Serial.println(F("ha movido durante la medida, o que esta averiada."));
  } else {
    Serial.println(F("FALLO: no se pudo leer la IMU."));
  }
  Serial.println();
}

void verCrudos() {
  if (!imuOk) {
    Serial.println(F("La IMU no esta lista. Usa 's' primero."));
    return;
  }
  int16_t acc[3], gyr[3];
  if (!leerCrudo(acc, gyr)) {
    Serial.println(F("FALLO al leer."));
    return;
  }
  Serial.println();
  Serial.println(F("=== VALORES CRUDOS ==="));
  Serial.print(F("  acelerometro (g) : X "));
  Serial.print((float)acc[0] / 16384.0f, 2);
  Serial.print(F("   Y "));
  Serial.print((float)acc[1] / 16384.0f, 2);
  Serial.print(F("   Z "));
  Serial.println((float)acc[2] / 16384.0f, 2);
  Serial.print(F("  giroscopo (deg/s): X "));
  Serial.print(((float)gyr[0] - sesgoGiro[0]) / 131.0f, 1);
  Serial.print(F("   Y "));
  Serial.print(((float)gyr[1] - sesgoGiro[1]) / 131.0f, 1);
  Serial.print(F("   Z "));
  Serial.println(((float)gyr[2] - sesgoGiro[2]) / 131.0f, 1);
  Serial.println();
  Serial.println(F("  Con el robot quieto y horizontal, un eje del"));
  Serial.println(F("  acelerometro deberia marcar cerca de 1.00 (la"));
  Serial.println(F("  gravedad) y los otros dos cerca de 0.00."));
  Serial.println();
}

/* Pide al usuario que incline el robot y comprueba si el angulo se
 * mueve en el sentido que dice el convenio del proyecto. */
void comprobarUnSigno(const char* titulo, const char* instruccion,
                      bool esRoll, const char* nombreDefine) {
  Serial.println();
  Serial.print(F("--- "));
  Serial.print(titulo);
  Serial.println(F(" ---"));
  Serial.println();
  Serial.println(F("1) Pon el robot HORIZONTAL y no lo toques."));
  Serial.println(F("   Escribe SI y pulsa Enter."));

  if (leerLinea(120000UL) != "SI") {
    Serial.println(F("Cancelado."));
    return;
  }

  float rollRef, pitchRef;
  if (!leerAngulos(&rollRef, &pitchRef)) {
    Serial.println(F("FALLO al leer la IMU."));
    return;
  }
  float referencia = esRoll ? rollRef : pitchRef;
  Serial.print(F("   Referencia en horizontal: "));
  Serial.println(referencia, 1);

  Serial.println();
  Serial.print(F("2) Ahora "));
  Serial.println(instruccion);
  Serial.println(F("   Mantenlo inclinado, escribe SI y pulsa Enter."));

  if (leerLinea(120000UL) != "SI") {
    Serial.println(F("Cancelado."));
    return;
  }

  float rollInc, pitchInc;
  if (!leerAngulos(&rollInc, &pitchInc)) {
    Serial.println(F("FALLO al leer la IMU."));
    return;
  }
  float inclinado = esRoll ? rollInc : pitchInc;
  float cambio = inclinado - referencia;

  Serial.print(F("   Inclinado: "));
  Serial.print(inclinado, 1);
  Serial.print(F("   -> ha cambiado "));
  Serial.print(cambio, 1);
  Serial.println(F(" grados"));
  Serial.println();

  if (fabsf(cambio) < 5.0f) {
    Serial.println(F("   *** APENAS HA CAMBIADO ***"));
    Serial.println(F("   O no lo has inclinado bastante (prueba con mas"));
    Serial.println(F("   de 15 grados), o la IMU esta montada en otro eje"));
    Serial.println(F("   distinto al que supone el firmware."));
  } else if (cambio > 0.0f) {
    Serial.println(F("   OK: el angulo ha AUMENTADO, que es lo correcto."));
    Serial.print(F("   Deja "));
    Serial.print(nombreDefine);
    Serial.println(F(" como esta en config.h."));
  } else {
    Serial.println(F("   *** SIGNO AL REVES ***"));
    Serial.println(F("   El angulo ha DISMINUIDO cuando deberia aumentar."));
    Serial.print(F("   Hay que invertir "));
    Serial.print(nombreDefine);
    Serial.println(F(" en config.h:"));
    Serial.print(F("     #define "));
    Serial.print(nombreDefine);
    Serial.println(F("  (-1.0f)"));
  }
}

void verificarSignos() {
  if (!imuOk) {
    Serial.println(F("La IMU no esta lista. Usa 's' primero."));
    return;
  }

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  VERIFICACION DE SIGNOS"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("Vamos a comprobar dos cosas, una por una:"));
  Serial.println(F("  1. Que al inclinar a la DERECHA, el roll AUMENTA"));
  Serial.println(F("  2. Que al inclinar hacia ATRAS, el pitch AUMENTA"));
  Serial.println();
  Serial.println(F("Puedes hacerlo levantando el robot a mano o poniendo"));
  Serial.println(F("un taco bajo un lado. Con 15-30 grados basta."));
  Serial.println();
  Serial.println(F("Este es el convenio del TFG (figura 7.40) y es el que"));
  Serial.println(F("espera el algoritmo de nivelado del modo 5."));

  comprobarUnSigno("ROLL (inclinacion lateral)",
                   "inclina el robot hacia la DERECHA",
                   true, "SIGNO_ROLL");

  comprobarUnSigno("PITCH (inclinacion adelante/atras)",
                   "inclina el robot hacia ATRAS (morro arriba)",
                   false, "SIGNO_PITCH");

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  FIN DE LA VERIFICACION"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("Si alguno de los dos ha salido al reves, cambialo en"));
  Serial.println(F("config.h, vuelve a subir este sketch y repite la"));
  Serial.println(F("prueba hasta que los dos salgan OK."));
  Serial.println();
  Serial.println(F("NO uses el modo 5 (nivelado automatico) con el robot"));
  Serial.println(F("en el suelo hasta que los dos signos sean correctos."));
  Serial.println();
}

/* ---- ARRANQUE -------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - PRUEBA 4 de 4: IMU"));
  Serial.println(F("=================================================="));
  Serial.println();
  Serial.println(F("Esta prueba NO mueve ningun motor. Es segura."));
  Serial.println(F("Sirve para comprobar el sensor de inclinacion."));

  buscarIMU();
  ayuda();
}

void loop() {
  if (modoContinuo) {
    float roll, pitch;
    if (leerAngulos(&roll, &pitch)) {
      Serial.print(F("  roll "));
      if (roll >= 0) Serial.print(F("+"));
      Serial.print(roll, 1);
      Serial.print(F("      pitch "));
      if (pitch >= 0) Serial.print(F("+"));
      Serial.println(pitch, 1);
    } else {
      Serial.println(F("  (fallo de lectura)"));
    }
    delay(200);

    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      modoContinuo = false;
      Serial.println(F(">>> Parado. Escribe 'h' para ver los comandos."));
      Serial.println();
    }
    return;
  }

  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  char c = linea.charAt(0);

  switch (c) {
    case 's': case 'S':
      buscarIMU();
      break;

    case 'c': case 'C':
      if (!imuOk) {
        Serial.println(F("La IMU no esta lista. Usa 's' primero."));
        break;
      }
      Serial.println();
      Serial.println(F("=== ROLL Y PITCH EN CONTINUO ==="));
      Serial.println(F("Inclina el robot y mira como cambian."));
      Serial.println(F("Convenio: roll + = a la derecha, pitch + = atras."));
      Serial.println(F("Pulsa Enter para parar."));
      Serial.println();
      modoContinuo = true;
      break;

    case 'v': case 'V':
      verificarSignos();
      break;

    case 'g': case 'G':
      calibrarGiro();
      break;

    case 'r': case 'R':
      verCrudos();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      break;
  }
}
