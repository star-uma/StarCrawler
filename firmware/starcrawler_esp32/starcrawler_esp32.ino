/*
 * starcrawler_esp32.ino
 * =====================
 * Firmware UNIFICADO de StarCrawler: un único ESP32 controla todo el robot.
 * Sustituye a la pareja Arduino MKR WiFi 1010 + ESP32 del TFG original.
 *
 * Responsabilidades:
 *   - WiFi/UDP directo con el PC (StarCrawlerXbox.py, sin cambios en el PC)
 *   - Modo 1: tracción diferencial -> 4x RMD-X8 por CAN a 1 Mbps
 *   - Modos 2/3/4: elevación -> 4x steppers DM542 + encoders AS5600
 *   - Modo 5: nivelado automático con IMU MPU9250 (Chart 7.4 del TFG)
 *   - Watchdog de seguridad (500 ms) y telemetría hacia el PC a 10 Hz
 *
 * Datagrama PC -> robot (18 bytes, 9 x int16 LE), idéntico al original:
 *   [0] id=1, [1] vel izq, [2] vel der, [3..6] o0..o3, [7] modo, [8] error
 *
 * Telemetría robot -> PC (18 bytes, 9 x int16 LE) al puerto 8886:
 *   [0] id=2, [1..4] ángulos*100 {FR,FL,RR,RL}, [5] roll*100,
 *   [6] pitch*100, [7] modo activo, [8] bits de error
 *
 * Compilar:
 *   arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "control_core.h"
#include "can_bus.h"
#include "steppers.h"
#include "sensors.h"

/* ─── Estado global ──────────────────────────────────────────────────────── */

WiFiUDP udp;

static cc_DatagramaPC rx;                 /* último datagrama válido del PC */
static IPAddress ipPC;                    /* a quién enviar la telemetría */
static bool pcConocido = false;

static uint32_t ultimoPaqueteMs = 0;
static bool enSeguridad = true;           /* arrancamos en estado seguro */

static int16_t modoActivo = 0;            /* 0 = ninguno / seguridad */

/* Tracción */
static float velIzqActual = 0.0f;
static float velDerActual = 0.0f;

/* Elevación */
static float angulos[CC_NUM_ORUGAS] = {180, 180, 180, 180};
static bool encoderOk[CC_NUM_ORUGAS] = {false, false, false, false};
static float objetivos[CC_NUM_ORUGAS] = {180, 180, 180, 180};
static bool enMarcha[CC_NUM_ORUGAS] = {false, false, false, false};
static int8_t ultimoCmd[CC_NUM_ORUGAS] = {0, 0, 0, 0};

/* IMU */
static cc_FiltroActitud actitud;
static bool imuOk = false;

/* Diagnóstico */
static bool canOk = false;
static uint32_t contadorCiclos = 0;

/* ─── Tracción: helpers CAN ──────────────────────────────────────────────── */

static void enviarVelocidadRMD(uint32_t id, float dps) {
  uint8_t trama[8];
  cc_tramaVelocidadRMD(dps, trama);
  canbus_enviar(id, trama);
  delayMicroseconds(CAN_INTER_FRAME_US);
}

static void liberarTraccion() {
  uint8_t trama[8];
  cc_tramaLiberarRMD(trama);
  const uint32_t ids[4] = {CAN_ID_FL, CAN_ID_RL, CAN_ID_FR, CAN_ID_RR};
  for (int i = 0; i < 4; i++) {
    canbus_enviar(ids[i], trama);
    delayMicroseconds(CAN_INTER_FRAME_US);
  }
}

/* Compensación de tracción durante la elevación (modos 2/3/4/5).
 * Regla extraída de los charts del modelo Simulink del MKR:
 *   oruga subiendo (ángulo disminuye, cmd=-1) -> s.h  (+COMPENSACION_DPS)
 *   oruga bajando  (ángulo aumenta,  cmd=+1) -> s.ah (-COMPENSACION_DPS)
 *   oruga parada   (cmd=0)                   -> velocidad 0 (frenado)
 */
static void compensarTraccion() {
#if COMPENSACION_TRACCION
  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS != 0) return;

  static const uint32_t idPorOruga[CC_NUM_ORUGAS] = {
      CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL}; /* orden {FR,FL,RR,RL} */
  static const float signo[CC_NUM_ORUGAS] = TABLA_SIGNO_COMPENSACION;

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float v = 0.0f;
    if (ultimoCmd[i] > 0) v = -COMPENSACION_DPS;      /* bajando -> s.ah */
    else if (ultimoCmd[i] < 0) v = COMPENSACION_DPS;  /* subiendo -> s.h */
    enviarVelocidadRMD(idPorOruga[i], v * signo[i]);
  }
#endif
}

/* ─── Seguridad ──────────────────────────────────────────────────────────── */

static void activarSeguridad(const char *motivo) {
  if (!enSeguridad) {
    Serial.print(F("[SEGURIDAD] "));
    Serial.println(motivo);
  }
  liberarTraccion();
  steppers_pararTodos();
  velIzqActual = 0.0f;
  velDerActual = 0.0f;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    enMarcha[i] = false;
    ultimoCmd[i] = 0;
  }
  modoActivo = 0;
  enSeguridad = true;
}

/* ─── WiFi ───────────────────────────────────────────────────────────────── */

static void conectarWiFi() {
#if USAR_IP_ESTATICA
  WiFi.config(IPAddress(IP_ROBOT), IPAddress(IP_PUERTA), IPAddress(IP_MASCARA));
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(RED_SSID, RED_PASS);

  Serial.print(F("[WiFi] Conectando a '" RED_SSID "'"));
  uint32_t inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000) {
    delay(500);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F(" OK - IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F(" FALLO (se reintenta en el lazo)"));
  }
}

/* Reconexión no bloqueante, como el firmware MKR de StarCrawler */
static void gestionarWiFi() {
  static uint32_t ultimoCheck = 0;
  if (millis() - ultimoCheck < 2000) return;
  ultimoCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    activarSeguridad("WiFi perdido");
    WiFi.disconnect();
    WiFi.begin(RED_SSID, RED_PASS);
  }
}

/* ─── Recepción UDP ──────────────────────────────────────────────────────── */

/* Vacía el buffer UDP quedándose con el último datagrama válido del PC. */
static bool leerUDP() {
  bool hayDatos = false;
  uint8_t buf[32];

  int tam = udp.parsePacket();
  while (tam > 0) {
    int leidos = udp.read(buf, sizeof(buf));
    cc_DatagramaPC tmp;
    if (cc_parseDatagrama(buf, leidos, &tmp) && tmp.id == 1) {
      rx = tmp;
      ipPC = udp.remoteIP();
      pcConocido = true;
      hayDatos = true;
    }
    tam = udp.parsePacket();
  }
  return hayDatos;
}

/* ─── Modos de control ───────────────────────────────────────────────────── */

static void modoTraccion() {
  float consignaIzq = cc_saturar((float)rx.action_left_train / 100.0f, VEL_MAX_DPS);
  float consignaDer = cc_saturar((float)rx.action_right_train / 100.0f, VEL_MAX_DPS);

  velIzqActual = cc_rateLimiter(velIzqActual, consignaIzq, RATE_LIMIT_DPS_CICLO);
  velDerActual = cc_rateLimiter(velDerActual, consignaDer, RATE_LIMIT_DPS_CICLO);

  /* Tramas a 50 Hz (cada 2 ciclos), con la inversión del tren izquierdo
   * y el espaciado entre tramas del firmware MKR original */
  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS == 0) {
    enviarVelocidadRMD(CAN_ID_FL, -velIzqActual);
    enviarVelocidadRMD(CAN_ID_RL, -velIzqActual);
    enviarVelocidadRMD(CAN_ID_FR, velDerActual);
    enviarVelocidadRMD(CAN_ID_RR, velDerActual);
  }
}

static void aplicarComandoOruga(int i, int8_t cmd) {
#if LIMITES_SOFTWARE_ACTIVOS
  cmd = cc_aplicarLimites(cmd, angulos[i], encoderOk[i],
                          ANGULO_MIN_DEG, ANGULO_MAX_DEG);
#endif
  steppers_comando(i, cmd);
  enMarcha[i] = (cmd != 0);
  ultimoCmd[i] = cmd;
}

static void modoPosicion(bool paqueteNuevo) {
  /* Los objetivos SOLO se actualizan con paquete fresco (el original leía
   * un array sin inicializar cuando no llegaba paquete en el ciclo). */
  if (paqueteNuevo) {
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
      objetivos[i] = (float)rx.o[i];
    }
  }

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (!encoderOk[i]) {
      /* Sin realimentación no hay control en lazo cerrado: motor parado.
       * (El original seguía moviéndose con el último ángulo congelado.) */
      aplicarComandoOruga(i, 0);
      continue;
    }
    int8_t cmd = cc_controlPosicion(angulos[i], objetivos[i], enMarcha[i],
                                    UMBRAL_ARRANQUE_DEG, UMBRAL_PARADA_DEG);
    aplicarComandoOruga(i, cmd);
  }
  compensarTraccion();
}

static void modoIncremental() {
  /* Modos 3 y 4: el PC ya envía el vector o[] con -1/0/+1 por oruga */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    int8_t cmd = (int8_t)((rx.o[i] > 0) - (rx.o[i] < 0));
    aplicarComandoOruga(i, cmd);
  }
  compensarTraccion();
}

static void modoNivelado() {
  if (!imuOk) {
    steppers_pararTodos();
    return;
  }
  int8_t Y[CC_NUM_ORUGAS];
  cc_nivelado(actitud.roll, actitud.pitch, LIMITE_NIVELADO_DEG, Y);
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    aplicarComandoOruga(i, Y[i]);
  }
  compensarTraccion();
}

/* Transición limpia al cambiar de modo */
static void gestionarCambioDeModo(int16_t nuevoModo) {
  if (nuevoModo == modoActivo) return;

  steppers_pararTodos();
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    enMarcha[i] = false;
    ultimoCmd[i] = 0;
  }

  if (modoActivo == 1) {
    velIzqActual = 0.0f;
    velDerActual = 0.0f;
  }
#if !COMPENSACION_TRACCION
  /* Sin compensación, fuera del modo 1 los RMD quedan sin par (original) */
  if (nuevoModo != 1) liberarTraccion();
#endif

  Serial.print(F("[MODO] Cambio a "));
  Serial.println(nuevoModo);
  modoActivo = nuevoModo;
}

/* ─── Telemetría ─────────────────────────────────────────────────────────── */

static uint16_t bitsDeError() {
  uint16_t e = 0;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (!encoderOk[i]) e |= (1u << i);
  }
  if (!imuOk) e |= CC_ERR_IMU;
  if (!canOk) e |= CC_ERR_CAN;
  if (enSeguridad) e |= CC_ERR_WATCHDOG;
  return e;
}

static void enviarTelemetria() {
  if (!pcConocido || WiFi.status() != WL_CONNECTED) return;

  uint8_t buf[18];
  cc_construirTelemetria(buf, angulos, actitud.roll, actitud.pitch,
                         modoActivo, bitsDeError());
  udp.beginPacket(ipPC, PUERTO_TELEMETRIA);
  udp.write(buf, sizeof(buf));
  udp.endPacket();
}

/* ─── Setup ──────────────────────────────────────────────────────────────── */

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║  StarCrawler ESP32 - control unificado ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));

  steppers_init();
  encoders_init();

  cc_filtroActitudReset(&actitud);
  imuOk = imu_init();
  if (imuOk) {
    Serial.println(F("[IMU] MPU9250 detectada. Calibrando gyro (robot quieto)..."));
    imu_calibrarGyro();
    Serial.println(F("[IMU] Calibracion completada."));
  } else {
    Serial.println(F("[IMU] MPU9250 NO detectada: modo 5 deshabilitado."));
  }

  canOk = canbus_init();
  if (canOk) {
    Serial.println(F("[CAN] Bus a 1 Mbps OK."));
    liberarTraccion(); /* estado inicial seguro, como el MKR original */
  } else {
    Serial.println(F("[CAN] ERROR: controlador no responde. Traccion deshabilitada."));
  }

  conectarWiFi();
  udp.begin(PUERTO_ROBOT);
  Serial.print(F("[UDP] Escuchando en puerto "));
  Serial.println(PUERTO_ROBOT);
  Serial.println(F("\nListo. Esperando paquetes de StarCrawlerXbox.py...\n"));
}

/* ─── Lazo principal (100 Hz) ────────────────────────────────────────────── */

void loop() {
  const uint32_t inicioCiclo = millis();
  contadorCiclos++;

  /* 1. Conectividad */
  gestionarWiFi();
  canbus_atender();

  /* 2. Entrada del PC */
  const bool paqueteNuevo = leerUDP();
  if (paqueteNuevo) {
    ultimoPaqueteMs = millis();
    if (enSeguridad) {
      Serial.println(F("[UDP] Enlace con el PC recuperado."));
      enSeguridad = false;
    }
  }

  /* 3. Watchdog */
  if (!enSeguridad &&
      cc_watchdogExpirado(millis(), ultimoPaqueteMs, WATCHDOG_TIMEOUT_MS)) {
    activarSeguridad("timeout de paquetes UDP");
  }

  /* 4. Sensores (siempre, también para telemetría) */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float ang;
    encoderOk[i] = encoders_leer(i, &ang);
    if (encoderOk[i]) angulos[i] = ang;
  }
  if (imuOk) {
    imuOk = imu_actualizar(&actitud, (float)CICLO_CONTROL_MS / 1000.0f);
  }

  /* 5. Control según modo */
  if (!enSeguridad) {
    gestionarCambioDeModo(rx.mode);
    switch (modoActivo) {
      case 1: modoTraccion(); break;
      case 2: modoPosicion(paqueteNuevo); break;
      case 3:
      case 4: modoIncremental(); break;
      case 5: modoNivelado(); break;
      default:
        steppers_pararTodos();
        break;
    }
  }

  /* 6. Telemetría a 10 Hz */
  if (contadorCiclos % TELEMETRIA_CADA_N_CICLOS == 0) {
    enviarTelemetria();
  }

  /* 7. Cadencia del lazo */
  const uint32_t transcurrido = millis() - inicioCiclo;
  if (transcurrido < CICLO_CONTROL_MS) {
    delay(CICLO_CONTROL_MS - transcurrido);
  }
}
