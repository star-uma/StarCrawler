/*
 * starcrawler_esp32_standalone.ino
 * ================================
 * Firmware STANDALONE de StarCrawler: robot completamente autónomo, SIN PC.
 * El mando Xbox se conecta por Bluetooth directamente al ESP32 (Bluepad32).
 *
 *   [Mando Xbox] --Bluetooth--> [ESP32] --CAN--> RMD-X8 (tracción)
 *                                       --GPIO-> DM542 (elevación)
 *                                       --I2C--> AS5600 x4 (encoders)
 *
 * La lógica que antes vivía en control/StarCrawlerXbox.py (deadzone, mezcla
 * diferencial, cambio de modo con RB, vectores de los modos 2/3/4) está
 * portada a gamepad_core.c y genera internamente el mismo datagrama que
 * enviaba el PC: el resto del firmware es idéntico a la variante básica.
 *
 * Modos (RB cicla 1->2->3->4->1; sin IMU no hay modo 5):
 *   1 Tracción diferencial   3 Incremental x4 (cruceta)
 *   2 Posición absoluta      4 Incremental x2 (cruceta + gatillos)
 *
 * Seguridad:
 *   - Mando desconectado -> parada inmediata (RMD liberados, steppers parados)
 *   - Mando conectado pero sin refrescar datos 500 ms -> parada
 *
 * TOOLCHAIN: requiere el board package de Bluepad32 (NO el core esp32 normal):
 *   arduino-cli core install esp32-bluepad32:esp32 --additional-urls
 *     https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
 *   arduino-cli lib install Bluepad32
 *
 * Compilar:
 *   arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_standalone
 *
 * Emparejar el mando: encenderlo y mantener el botón de sincronización hasta
 * que parpadee rápido; el ESP32 lo detecta solo (queda emparejado para
 * siempre). Requiere mando Xbox con Bluetooth (One S / modelo 1708 o
 * posterior; si se empareja con un móvil, vale).
 */
#include <Bluepad32.h>

#include "config.h"
#include "control_core.h"
#include "gamepad_core.h"
#include "can_bus.h"
#include "steppers.h"
#include "encoders.h"

/* ─── Estado global ──────────────────────────────────────────────────────── */

static ControllerPtr mando = nullptr;

static gc_EstadoControl estadoMando;
static gc_EstadoSimultaneo estadoSim;
static cc_DatagramaPC rx; /* datagrama sintetizado desde el mando */

static uint32_t ultimoDatoMandoMs = 0;
static bool enSeguridad = true;

static int16_t modoActivo = 0;

/* Tracción */
static float velIzqActual = 0.0f;
static float velDerActual = 0.0f;

/* Elevación */
static float angulos[CC_NUM_ORUGAS] = {180, 180, 180, 180};
static bool encoderOk[CC_NUM_ORUGAS] = {false, false, false, false};
static float objetivos[CC_NUM_ORUGAS] = {180, 180, 180, 180};
static bool enMarcha[CC_NUM_ORUGAS] = {false, false, false, false};
static int8_t ultimoCmd[CC_NUM_ORUGAS] = {0, 0, 0, 0};

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

static void compensarTraccion() {
#if COMPENSACION_TRACCION
  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS != 0) return;

  static const uint32_t idPorOruga[CC_NUM_ORUGAS] = {
      CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL};
  static const float signo[CC_NUM_ORUGAS] = TABLA_SIGNO_COMPENSACION;

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float v = 0.0f;
    if (ultimoCmd[i] > 0) v = -COMPENSACION_DPS;
    else if (ultimoCmd[i] < 0) v = COMPENSACION_DPS;
    enviarVelocidadRMD(idPorOruga[i], v * signo[i]);
  }
#endif
}

/* ─── Seguridad ──────────────────────────────────────────────────────────── */

static void activarSeguridad(const char *motivo) {
  if (!enSeguridad) {
    Serial.print("[SEGURIDAD] ");
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

/* ─── Mando (Bluepad32) ──────────────────────────────────────────────────── */

static void alConectarMando(ControllerPtr ctl) {
  if (mando == nullptr) {
    mando = ctl;
    Serial.print("[MANDO] Conectado: ");
    Serial.println(ctl->getModelName().c_str());
  } else {
    Serial.println("[MANDO] Segundo mando ignorado.");
  }
}

static void alDesconectarMando(ControllerPtr ctl) {
  if (mando == ctl) {
    mando = nullptr;
    Serial.println("[MANDO] Desconectado.");
    activarSeguridad("mando desconectado");
  }
}

/* Convierte el estado Bluepad32 al formato normalizado de gamepad_core */
static void leerMando(gc_EstadoMando *m) {
  /* Ejes: Bluepad32 da [-512..511]; adelante = eje Y negativo (como pygame) */
  m->ejeIzqY = gc_deadzone(-(float)mando->axisY() / FONDO_ESCALA_EJE, ZONA_MUERTA);
  m->ejeDerX = gc_deadzone((float)mando->axisRX() / FONDO_ESCALA_EJE, ZONA_MUERTA);

  m->rb = mando->r1();
  m->a = mando->a();
  m->b = mando->b();
  m->x = mando->x();
  m->y = mando->y();

  const uint8_t dpad = mando->dpad();
  m->dpadArriba = dpad & DPAD_UP;
  m->dpadAbajo = dpad & DPAD_DOWN;
  m->dpadIzq = dpad & DPAD_LEFT;
  m->dpadDer = dpad & DPAD_RIGHT;

  /* Gatillos 0..1023: LT = brake (subir), RT = throttle (bajar) */
  m->trigIzq = mando->brake() > (int)(UMBRAL_TRIGGER * 1023);
  m->trigDer = mando->throttle() > (int)(UMBRAL_TRIGGER * 1023);

  /* Botones adicionales del esquema simultáneo */
  m->l1 = mando->l1();
  m->l2 = mando->l2();
  m->r2 = mando->r2();
  m->l3 = mando->thumbL();
  m->share = mando->miscSelect();
  m->options = mando->miscStart();
}

/* ─── Modos de control (idénticos a la variante básica) ─────────────────── */

static void modoTraccion() {
  float consignaIzq = cc_saturar((float)rx.action_left_train / 100.0f, VEL_MAX_DPS);
  float consignaDer = cc_saturar((float)rx.action_right_train / 100.0f, VEL_MAX_DPS);

  velIzqActual = cc_rateLimiter(velIzqActual, consignaIzq, RATE_LIMIT_DPS_CICLO);
  velDerActual = cc_rateLimiter(velDerActual, consignaDer, RATE_LIMIT_DPS_CICLO);

  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS == 0) {
    enviarVelocidadRMD(CAN_ID_FL, -velIzqActual); /* tren izq. invertido */
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

static void modoPosicion() {
  /* Con el mando a bordo el objetivo siempre es fresco (gc_procesar
   * persiste la última consigna de A/B/X/Y) */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    objetivos[i] = (float)rx.o[i];
  }

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (!encoderOk[i]) {
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
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    int8_t cmd = (int8_t)((rx.o[i] > 0) - (rx.o[i] < 0));
    aplicarComandoOruga(i, cmd);
  }
  compensarTraccion();
}

/* ─── Esquema simultáneo: tracción + orugas a la vez ─────────────────────── */

/* Envía a cada RMD la velocidad de su tren MÁS la compensación de su oruga
 * (en este esquema conducir y bascular conviven, así que se suman). */
static void enviarTraccionCompensada() {
  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS != 0) return;

  static const uint32_t idPorOruga[CC_NUM_ORUGAS] = {
      CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL}; /* orden {FR,FL,RR,RL} */
  static const float signo[CC_NUM_ORUGAS] = TABLA_SIGNO_COMPENSACION;
  /* Tren izquierdo invertido, como siempre */
  const float velLado[CC_NUM_ORUGAS] = {velDerActual, -velIzqActual,
                                        velDerActual, -velIzqActual};
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float comp = 0.0f;
#if COMPENSACION_TRACCION
    if (ultimoCmd[i] > 0) comp = -COMPENSACION_DPS;
    else if (ultimoCmd[i] < 0) comp = COMPENSACION_DPS;
    comp *= signo[i];
#endif
    enviarVelocidadRMD(idPorOruga[i], velLado[i] + comp);
  }
}

static void controlSimultaneo() {
  gc_EstadoMando m;
  leerMando(&m);
  gc_SalidaSimultanea sal;
  gc_procesarSimultaneo(&estadoSim, &m, millis(), GANANCIA_JOYSTICK,
                        VEL_MAX_DPS, &sal);

  if (sal.paradaEmergencia) {
    if (modoActivo != 0) Serial.println("[MANDO] PARADA DE EMERGENCIA (SHARE)");
    liberarTraccion();
    steppers_pararTodos();
    velIzqActual = 0.0f;
    velDerActual = 0.0f;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
      enMarcha[i] = false;
      ultimoCmd[i] = 0;
    }
    modoActivo = 0;
    return;
  }
  modoActivo = 6; /* 6 = esquema simultáneo (para telemetría/dashboard) */

  /* Tracción con rampa, siempre activa */
  velIzqActual = cc_rateLimiter(
      velIzqActual, cc_saturar((float)sal.action_left_train / 100.0f, VEL_MAX_DPS),
      RATE_LIMIT_DPS_CICLO);
  velDerActual = cc_rateLimiter(
      velDerActual, cc_saturar((float)sal.action_right_train / 100.0f, VEL_MAX_DPS),
      RATE_LIMIT_DPS_CICLO);

  /* Orugas: manual incremental o preset de posición */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    int8_t cmd;
    if (sal.usarPosicion) {
      cmd = encoderOk[i]
                ? cc_controlPosicion(angulos[i], (float)sal.objetivo[i],
                                     enMarcha[i], UMBRAL_ARRANQUE_DEG,
                                     UMBRAL_PARADA_DEG)
                : 0; /* sin encoder no hay lazo cerrado */
    } else {
      cmd = sal.oruga[i];
    }
    aplicarComandoOruga(i, cmd);
  }

  enviarTraccionCompensada();
}

/* ─── Esquema TFG: gestión de modos ──────────────────────────────────────── */

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
  if (nuevoModo != 1) liberarTraccion();
#endif

  Serial.print("[MODO] Cambio a ");
  Serial.println(nuevoModo);
  modoActivo = nuevoModo;
}

/* ─── Telemetría por serie (10 Hz) ───────────────────────────────────────── */

static uint16_t bitsDeError() {
  uint16_t e = 0;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (!encoderOk[i]) e |= (1u << i);
  }
  if (!canOk) e |= CC_ERR_CAN;
  if (enSeguridad) e |= CC_ERR_WATCHDOG;
  return e;
}

/* Línea parseable por control/StarCrawlerDashboard.py (y legible a ojo):
 * TLM,<modo>,<velIzq>,<velDer>,<angFR>,<angFL>,<angRR>,<angRL>,<errores> */
static void emitirTelemetriaSerie() {
  Serial.printf("TLM,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u\n",
                (int)modoActivo, velIzqActual, velDerActual,
                angulos[0], angulos[1], angulos[2], angulos[3],
                (unsigned)bitsDeError());
}

/* ─── Setup ──────────────────────────────────────────────────────────────── */

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║  StarCrawler ESP32 STANDALONE (sin PC)        ║");
  Serial.println("║  Mando Xbox por Bluetooth directo (Bluepad32) ║");
  Serial.println("╚══════════════════════════════════════════════╝");

  steppers_init();
  encoders_init();

  canOk = canbus_init();
  if (canOk) {
    Serial.println("[CAN] Bus a 1 Mbps OK.");
    liberarTraccion();
  } else {
    Serial.println("[CAN] ERROR: controlador no responde. Traccion deshabilitada.");
  }

  gc_reset(&estadoMando);
  gc_simultaneoReset(&estadoSim);
#if ESQUEMA_CONTROL == ESQUEMA_SIMULTANEO
  Serial.println("[CTRL] Esquema SIMULTANEO: traccion siempre activa,");
  Serial.println("       L1/L2 par delantero, R1/R2 trasero, SHARE = emergencia.");
#else
  Serial.println("[CTRL] Esquema MODOS TFG: R1 cicla 1->2->3->4->1.");
#endif

  /* Bluepad32: callbacks de conexión. Las claves de emparejamiento se
   * conservan entre reinicios (no llamar a forgetBluetoothKeys salvo para
   * re-emparejar un mando nuevo). */
  BP32.setup(&alConectarMando, &alDesconectarMando);
  BP32.enableVirtualDevice(false);

  Serial.println("\nListo. Pon el mando Xbox en modo emparejamiento");
  Serial.println("(boton de sincronizacion hasta parpadeo rapido).\n");
}

/* ─── Lazo principal (100 Hz) ────────────────────────────────────────────── */

void loop() {
  const uint32_t inicioCiclo = millis();
  contadorCiclos++;

  /* 1. Refrescar datos del mando */
  const bool datosNuevos = BP32.update();
  if (datosNuevos && mando != nullptr && mando->isConnected()) {
    ultimoDatoMandoMs = millis();
    if (enSeguridad) {
      Serial.println("[MANDO] Control recuperado.");
      enSeguridad = false;
    }
  }

  /* 2. Watchdog: sin mando o sin datos frescos -> parada */
  if (!enSeguridad) {
    if (mando == nullptr || !mando->isConnected()) {
      activarSeguridad("mando no conectado");
    } else if (cc_watchdogExpirado(millis(), ultimoDatoMandoMs,
                                   WATCHDOG_TIMEOUT_MS)) {
      activarSeguridad("mando sin refrescar datos");
    }
  }

  /* 3. Encoders */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float ang;
    encoderOk[i] = encoders_leer(i, &ang);
    if (encoderOk[i]) angulos[i] = ang;
  }

  /* 4. Mando -> control */
  if (!enSeguridad) {
#if ESQUEMA_CONTROL == ESQUEMA_SIMULTANEO
    controlSimultaneo();
#else
    /* Esquema clásico del TFG: mando -> datagrama -> modos */
    gc_EstadoMando m;
    leerMando(&m);
    gc_procesar(&estadoMando, &m, GANANCIA_JOYSTICK, VEL_MAX_DPS, &rx);

    gestionarCambioDeModo(rx.mode);
    switch (modoActivo) {
      case 1: modoTraccion(); break;
      case 2: modoPosicion(); break;
      case 3:
      case 4: modoIncremental(); break;
      default:
        steppers_pararTodos();
        for (int i = 0; i < CC_NUM_ORUGAS; i++) ultimoCmd[i] = 0;
        break;
    }
#endif
  }

  /* 5. Telemetría por serie a 10 Hz */
  if (contadorCiclos % TELEMETRIA_SERIE_CADA_N_CICLOS == 0) {
    emitirTelemetriaSerie();
  }

  /* 6. Cadencia del lazo */
  const uint32_t transcurrido = millis() - inicioCiclo;
  if (transcurrido < CICLO_CONTROL_MS) {
    delay(CICLO_CONTROL_MS - transcurrido);
  }
}
