/*
 * starcrawler_esp32_ros2.ino
 * ==========================
 * Firmware del ESP32 para la arquitectura con PC a bordo y ROS 2.
 *
 *   [Mando] --> [PC a bordo: Ubuntu + ROS 2] --USB serie--> [ESP32]
 *                    joy -> teleop -> driver                   |
 *                                                              |-- CAN 1 Mbps -> 4x RMD-X8
 *                                                              |-- GPIO -> 4x DM542
 *                                                              |-- I2C -> TCA9548A + 4x AS5600
 *
 * El ESP32 NO decide nada de alto nivel: ejecuta consignas y protege el
 * hardware. Se queda con lo que necesita tiempo real (pulsos de stepper a
 * 833 Hz, lazo de posicion a 100 Hz, tramas CAN espaciadas) porque un Linux
 * sin kernel RT no lo hace bien.
 *
 * Protocolo: ver proto.h (identico a starcrawler_driver/protocol.py).
 *   PC -> ESP32: velocidades de tren + incrementos u objetivos de oruga
 *   ESP32 -> PC: angulos, velocidades, bits de error, flag de seguridad
 *
 * Seguridad: watchdog de 300 ms sobre las tramas del PC + flag de emergencia.
 * Si el PC se cuelga o se desconecta el USB, el robot para solo. Esta capa
 * nunca se delega al PC.
 *
 * Compilar (core esp32 estandar, NO el de bluepad32):
 *   arduino-cli lib install ACAN2515
 *   arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_ros2
 */
#include "config.h"
#include "control_core.h"
#include "proto.h"
#include "can_bus.h"
#include "steppers.h"
#include "encoders.h"

/* ─── Estado global ──────────────────────────────────────────────────────── */

static sp_Parser parser;
static sp_Comando cmd;                  /* ultima consigna valida del PC */

static uint32_t ultimaTramaMs = 0;
static bool enSeguridad = true;         /* arrancamos en estado seguro */

/* Traccion */
static float velIzqActual = 0.0f;
static float velDerActual = 0.0f;

/* Elevacion */
static float angulos[CC_NUM_ORUGAS] = {180, 180, 180, 180};
static bool encoderOk[CC_NUM_ORUGAS] = {false, false, false, false};
static bool enMarcha[CC_NUM_ORUGAS] = {false, false, false, false};
static int8_t ultimoCmd[CC_NUM_ORUGAS] = {0, 0, 0, 0};

static bool canOk = false;
static uint32_t contadorCiclos = 0;

/* ─── CAN ────────────────────────────────────────────────────────────────── */

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

/* Velocidad de cada tren MAS la compensacion de su oruga (se suman: el PC
 * puede pedir conducir y bascular a la vez). */
static void enviarTraccion() {
  if (contadorCiclos % ENVIO_CAN_CADA_N_CICLOS != 0) return;

  static const uint32_t idPorOruga[CC_NUM_ORUGAS] = {
      CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL}; /* orden {FR,FL,RR,RL} */
  static const float signo[CC_NUM_ORUGAS] = TABLA_SIGNO_COMPENSACION;
  /* Tren izquierdo invertido por la disposicion mecanica */
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

/* ─── Seguridad ──────────────────────────────────────────────────────────── */

static void activarSeguridad() {
  liberarTraccion();
  steppers_pararTodos();
  velIzqActual = 0.0f;
  velDerActual = 0.0f;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    enMarcha[i] = false;
    ultimoCmd[i] = 0;
  }
  enSeguridad = true;
}

/* ─── Recepcion ──────────────────────────────────────────────────────────── */

static void leerSerie() {
  uint8_t payload[SP_LEN_CMD];
  while (Serial.available() > 0) {
    const int b = Serial.read();
    if (b < 0) break;
    if (sp_parserByte(&parser, (uint8_t)b, payload)) {
      sp_desempaquetarComando(payload, &cmd);
      ultimaTramaMs = millis();
      enSeguridad = false;
    }
  }
}

/* ─── Control ────────────────────────────────────────────────────────────── */

static void aplicarComandoOruga(int i, int8_t c) {
#if LIMITES_SOFTWARE_ACTIVOS
  c = cc_aplicarLimites(c, angulos[i], encoderOk[i],
                        ANGULO_MIN_DEG, ANGULO_MAX_DEG);
#endif
  steppers_comando(i, c);
  enMarcha[i] = (c != 0);
  ultimoCmd[i] = c;
}

static void ejecutarConsigna() {
  /* Traccion con rampa (segunda capa de saturacion: el PC ya limita) */
  const float consignaIzq =
      cc_saturar((float)cmd.vel_izq_cdps / 100.0f, VEL_MAX_DPS);
  const float consignaDer =
      cc_saturar((float)cmd.vel_der_cdps / 100.0f, VEL_MAX_DPS);
  velIzqActual = cc_rateLimiter(velIzqActual, consignaIzq, RATE_LIMIT_DPS_CICLO);
  velDerActual = cc_rateLimiter(velDerActual, consignaDer, RATE_LIMIT_DPS_CICLO);

  const bool usarPosicion = (cmd.flags & SP_FLAG_CMD_USAR_POSICION) != 0;

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    int8_t c;
    if (usarPosicion) {
      /* Sin encoder valido no hay lazo cerrado: motor parado */
      c = encoderOk[i]
              ? cc_controlPosicion(angulos[i],
                                   (float)cmd.objetivo_cdeg[i] / 100.0f,
                                   enMarcha[i], UMBRAL_ARRANQUE_DEG,
                                   UMBRAL_PARADA_DEG)
              : 0;
    } else {
      const int8_t bruto = cmd.incremento[i];
      c = (int8_t)((bruto > 0) - (bruto < 0));
    }
    aplicarComandoOruga(i, c);
  }

  enviarTraccion();
}

/* ─── Telemetria ─────────────────────────────────────────────────────────── */

static void enviarEstado() {
  sp_Estado st;
  uint16_t errores = 0;
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    st.angulo_cdeg[i] = (int16_t)lrintf(angulos[i] * 100.0f);
    if (!encoderOk[i]) errores |= (1u << i);
  }
  if (!canOk) errores |= CC_ERR_CAN;
  if (enSeguridad) errores |= CC_ERR_WATCHDOG;

  st.vel_izq_cdps = (int16_t)lrintf(velIzqActual * 100.0f);
  st.vel_der_cdps = (int16_t)lrintf(velDerActual * 100.0f);
  st.error_bits = errores;
  st.flags = enSeguridad ? SP_FLAG_ST_SEGURIDAD : 0;
  st.seq_eco = cmd.seq;
  st.roll_cdeg = 0;   /* reservado: sin IMU en esta variante */
  st.pitch_cdeg = 0;

  uint8_t trama[SP_TAM_TRAMA_STATE];
  const int n = sp_empaquetarEstado(&st, trama);
  Serial.write(trama, n);
}

/* ─── Setup ──────────────────────────────────────────────────────────────── */

void setup() {
  Serial.begin(SERIE_BAUDIOS);
  delay(200);

#if DEBUG_TEXTO
  Serial.println();
  Serial.println("StarCrawler ESP32 - esclavo serie para ROS 2");
#endif

  sp_parserInit(&parser, SP_TIPO_CMD, SP_LEN_CMD);
  memset(&cmd, 0, sizeof(cmd));
  for (int i = 0; i < CC_NUM_ORUGAS; i++) cmd.objetivo_cdeg[i] = 18000;

  steppers_init();
  encoders_init();

  canOk = canbus_init();
  if (canOk) liberarTraccion();  /* estado inicial seguro */

  ultimaTramaMs = millis();
}

/* ─── Lazo principal (100 Hz) ────────────────────────────────────────────── */

void loop() {
  const uint32_t inicioCiclo = millis();
  contadorCiclos++;

  canbus_atender();
  leerSerie();

  /* Watchdog y emergencia */
  const bool emergencia = (cmd.flags & SP_FLAG_CMD_EMERGENCIA) != 0;
  if (!enSeguridad &&
      (emergencia ||
       cc_watchdogExpirado(millis(), ultimaTramaMs, WATCHDOG_TIMEOUT_MS))) {
    activarSeguridad();
  }

  /* Encoders (siempre, tambien para telemetria) */
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    float ang;
    encoderOk[i] = encoders_leer(i, &ang);
    if (encoderOk[i]) angulos[i] = ang;
  }

  if (!enSeguridad) {
    ejecutarConsigna();
  }

  if (contadorCiclos % TELEMETRIA_CADA_N_CICLOS == 0) {
    enviarEstado();
  }

  const uint32_t transcurrido = millis() - inicioCiclo;
  if (transcurrido < CICLO_CONTROL_MS) {
    delay(CICLO_CONTROL_MS - transcurrido);
  }
}
