/*
 * test_can_esp32.ino - StarCrawler - CAN del ESP32 por TWAI
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   El bus CAN del ESP32 usando su controlador interno (TWAI) con un
 *   transceptor externo. Sirve para validar el transceptor antes de
 *   confiarle los motores de traccion.
 *
 *   RIESGO: NINGUNO con los comandos 's', 'i' y 'l'. No mueven motores.
 *
 *   CABLEADO (valores de firmware/starcrawler_esp32/config.h)
 *
 *     ESP32 GPIO 5  (TX)  ->  TXD del transceptor
 *     ESP32 GPIO 35 (RX)  <-  RXD del transceptor
 *     GND comun ENTRE AMBOS  <- imprescindible
 *     Transceptor CANH/CANL -> al bus, en paralelo con los motores
 *
 *   SEGUN EL TRANSCEPTOR
 *     SN65HVD230 (recomendado): VCC 3.3 V, RS a GND. Conexion directa.
 *     TJA1050 a 5 V          : RXD saca 5 V -> DIVISOR obligatorio hacia
 *                              el GPIO 35 (1.8k / 3.3k). Pin S a GND.
 *     TJA1050 a 3.3 V        : fuera de especificacion (pide 4.75-5.25 V).
 *                              No daña nada y la logica encaja sin divisor,
 *                              pero el nivel dominante en el bus sale flojo.
 *                              Justo para eso esta el comando 'i': mira los
 *                              contadores de error, no te fies de que
 *                              "parece que va".
 *
 *   IMPORTANTE
 *     No puede haber dos placas mandando en el bus a la vez. Si el Arduino
 *     MKR esta conectado al bus, desconectalo o no lo alimentes.
 *
 * ================================================================
 */

#include "driver/twai.h"

/* ---- CONFIGURACION --------------------------------------------- */
#define PIN_TWAI_TX  GPIO_NUM_5
#define PIN_TWAI_RX  GPIO_NUM_35

#define ID_PRIMERO  0x141
#define ID_ULTIMO   0x148

#define RMD_CMD_APAGAR    0x80
#define RMD_CMD_ESTADO1   0x9A   /* temp, tension, flags de error */
#define RMD_CMD_ESTADO2   0x9C   /* temp, corriente, velocidad, posicion */
#define RMD_CMD_VELOCIDAD 0xA2   /* control de velocidad en lazo cerrado */
#define RMD_CMD_ANG_MULTI 0x92   /* angulo multivuelta (0.01 grados) */

bool twaiListo = false;

/* ---- BAJO NIVEL ------------------------------------------------ */

/* No se encola nada si el bus no esta en marcha: si se acumulan tramas y el
 * bus cae, el driver TWAI de Espressif revienta con
 *   assert failed: twai_handle_tx_buffer_frame twai.c:184 (tx_msg_count >= 0)
 * y el ESP32 se reinicia. Reproducido apagando los motores en caliente. */
bool enviarTrama(uint32_t id, const uint8_t datos[8], uint32_t msEspera = 10) {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK || st.state != TWAI_STATE_RUNNING) {
    return false;
  }
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = 8;
  for (int i = 0; i < 8; i++) msg.data[i] = datos[i];
  return twai_transmit(&msg, pdMS_TO_TICKS(msEspera)) == ESP_OK;
}

/* Recuperacion de bus-off. El orden importa: primero vaciar las colas
 * (de ahi venia el assert), luego recuperar, y esperar al estado STOPPED
 * antes de arrancar de nuevo. */
bool recuperarDeBusOff() {
  twai_clear_transmit_queue();
  twai_clear_receive_queue();
  if (twai_initiate_recovery() != ESP_OK) return false;

  for (int i = 0; i < 150; i++) {
    delay(20);
    twai_status_info_t s;
    if (twai_get_status_info(&s) == ESP_OK && s.state == TWAI_STATE_STOPPED) {
      return twai_start() == ESP_OK;
    }
  }
  return false;
}

/* Espera respuesta del motor. El V3 contesta en id+0x100, el V2 en el mismo. */
int esperarRespuesta(uint32_t idPeticion, uint8_t datos[8], uint32_t msLimite) {
  uint32_t limite = millis() + msLimite;
  while (millis() < limite) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (msg.identifier == idPeticion + 0x100 || msg.identifier == idPeticion) {
        int n = msg.data_length_code;
        if (n > 8) n = 8;
        for (int i = 0; i < n; i++) datos[i] = msg.data[i];
        return n;
      }
    }
  }
  return 0;
}

void vaciarRx() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) { }
}

int16_t i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ---- COMANDOS --------------------------------------------------- */

void ayuda() {
  Serial.println();
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println(F("|  s   Escanear IDs en el bus          (no mueve)      |"));
  Serial.println(F("|  i   Estado del bus y CONTADORES DE ERROR            |"));
  Serial.println(F("|      <- lo importante para validar el transceptor    |"));
  Serial.println(F("|  l   Apagar todos los motores (quedan sueltos)       |"));
  Serial.println(F("|  t   Prueba de carga: 200 tramas seguidas            |"));
  Serial.println(F("|  m   MOVER el motor +10/-10 dps      (MUEVE!)        |"));
  Serial.println(F("|  w   Ciclo apagado/encendido: mide la sacudida y     |"));
  Serial.println(F("|      prueba la recuperacion de bus-off               |"));
  Serial.println(F("|  r   Recuperar el bus tras un bus-off                |"));
  Serial.println(F("|  h   Mostrar esta ayuda                              |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println();
}

const char* nombreEstado(twai_state_t e) {
  switch (e) {
    case TWAI_STATE_STOPPED:   return "PARADO";
    case TWAI_STATE_RUNNING:   return "EN MARCHA";
    case TWAI_STATE_BUS_OFF:   return "BUS-OFF (bus caido)";
    case TWAI_STATE_RECOVERING: return "RECUPERANDOSE";
    default:                   return "?";
  }
}

/* Los contadores de error son la clave para validar un transceptor:
 * un bus sano los mantiene a cero. Si suben, el nivel electrico no es
 * fiable aunque de vez en cuando llegue alguna trama. */
void estadoBus() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) {
    Serial.println(F("No se pudo leer el estado del TWAI."));
    return;
  }
  Serial.println();
  Serial.println(F("=== ESTADO DEL BUS CAN ==="));
  Serial.print(F("  estado            : ")); Serial.println(nombreEstado(st.state));
  Serial.print(F("  errores TX        : ")); Serial.println(st.tx_error_counter);
  Serial.print(F("  errores RX        : ")); Serial.println(st.rx_error_counter);
  Serial.print(F("  tramas en cola TX : ")); Serial.println(st.msgs_to_tx);
  Serial.print(F("  tramas en cola RX : ")); Serial.println(st.msgs_to_rx);
  Serial.print(F("  TX fallidas       : ")); Serial.println(st.tx_failed_count);
  Serial.print(F("  RX perdidas       : ")); Serial.println(st.rx_missed_count);
  Serial.print(F("  errores de bus    : ")); Serial.println(st.bus_error_count);
  Serial.print(F("  arbitrajes perdidos: ")); Serial.println(st.arb_lost_count);
  Serial.println();

  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.println(F("  El bus esta caido. Causas tipicas:"));
    Serial.println(F("    - el transceptor no llega a imponer el nivel dominante"));
    Serial.println(F("      (tipico si el TJA1050 va a 3.3 V en vez de 5 V)"));
    Serial.println(F("    - CANH y CANL cruzados o en corto"));
    Serial.println(F("    - no hay ningun otro nodo alimentado en el bus"));
    Serial.println(F("    - terminacion mal: deben ser 120 ohm SOLO en los extremos"));
    Serial.println(F("  Usa 'r' para intentar recuperarlo."));
  } else if (st.tx_error_counter > 0 || st.bus_error_count > 0) {
    Serial.println(F("  Hay errores contados: el bus NO esta fino."));
    Serial.println(F("  Con un transceptor en especificacion deberian ser 0."));
  } else {
    Serial.println(F("  Sin errores. El bus esta fino."));
  }
  Serial.println();
}

void escanearIDs() {
  Serial.println();
  Serial.println(F("=== ESCANEO DE IDs ==="));
  Serial.println(F("Se pregunta el estado a cada ID. No mueve nada."));
  Serial.println();

  int responden = 0;
  for (uint32_t id = ID_PRIMERO; id <= ID_ULTIMO; id++) {
    vaciarRx();
    uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};

    Serial.print(F("  0x"));
    Serial.print(id, HEX);
    Serial.print(F("  ->  "));

    if (!enviarTrama(id, pet)) {
      Serial.println(F("no se pudo ni enviar (bus caido: usa 'i')"));
      delay(50);
      continue;
    }

    uint8_t r[8];
    int n = esperarRespuesta(id, r, 60);
    if (n == 0) {
      Serial.println(F("SIN RESPUESTA"));
    } else {
      Serial.print(F("RESPONDE   vel "));
      Serial.print(i16(r + 4));
      Serial.print(F(" dps, temp "));
      Serial.print((int8_t)r[1]);
      Serial.println(F(" C"));
      responden++;
    }
    delay(60);
  }

  Serial.println();
  Serial.print(F("Responden "));
  Serial.print(responden);
  Serial.println(F(" motores (lo esperado son 4: 0x141..0x144)."));
  Serial.println(F("Mira ahora 'i': aunque respondan, los contadores de"));
  Serial.println(F("error dicen si el bus va justo."));
  Serial.println();
}

/* Manda muchas tramas seguidas: un transceptor marginal aguanta una
 * trama suelta pero se cae bajo carga. */
void pruebaCarga() {
  Serial.println();
  Serial.println(F("=== PRUEBA DE CARGA: 200 tramas ==="));

  twai_status_info_t antes;
  twai_get_status_info(&antes);

  uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
  int enviadas = 0, respondidas = 0;
  for (int i = 0; i < 200; i++) {
    vaciarRx();
    if (enviarTrama(ID_PRIMERO, pet, 5)) enviadas++;
    uint8_t r[8];
    if (esperarRespuesta(ID_PRIMERO, r, 10) > 0) respondidas++;
    delayMicroseconds(250);
  }

  twai_status_info_t despues;
  twai_get_status_info(&despues);

  Serial.print(F("  enviadas    : ")); Serial.println(enviadas);
  Serial.print(F("  respondidas : ")); Serial.println(respondidas);
  Serial.print(F("  perdidas    : ")); Serial.println(enviadas - respondidas);
  Serial.print(F("  errores de bus nuevos : "));
  Serial.println(despues.bus_error_count - antes.bus_error_count);
  Serial.print(F("  estado final: ")); Serial.println(nombreEstado(despues.state));
  Serial.println();
  Serial.println(F("Un bus sano: 200 respondidas y 0 errores nuevos."));
  Serial.println();
}

void apagarTodos() {
  uint8_t t[8] = {RMD_CMD_APAGAR, 0, 0, 0, 0, 0, 0, 0};
  for (uint32_t id = ID_PRIMERO; id <= ID_ULTIMO; id++) {
    enviarTrama(id, t);
    delayMicroseconds(250);
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

/* Mueve el primer motor que responda, despacio y con telemetria.
 * *** ESTE COMANDO MUEVE UN MOTOR: robot sobre tacos / motor amarrado *** */
void moverMotor() {
  Serial.println();
  Serial.println(F("=== MOVER MOTOR DESDE EL ESP32 ==="));
  Serial.println(F("*** ESTO MUEVE EL MOTOR DE VERDAD ***"));
  Serial.println(F("Comprueba: motor amarrado / robot sobre tacos, eje libre."));
  Serial.println();
  Serial.println(F("Secuencia: +10 dps 3 s, parada 1 s, -10 dps 3 s."));
  Serial.println(F("Escribe SI para empezar (otra cosa cancela)."));
  if (leerLinea(60000UL) != "SI") { Serial.println(F("Cancelado.")); return; }

  Serial.println(F("CSV: DAT,ms,consigna,vel,corriente_cA,temp,pos"));
  Serial.println(F("INICIO"));

  const float fases[3] = {10.0f, 0.0f, -10.0f};
  const unsigned long duracion[3] = {3000, 1000, 3000};
  unsigned long t0 = millis();

  for (int f = 0; f < 3; f++) {
    int32_t v = (int32_t)lround(fases[f] * 100.0);
    uint8_t trama[8];
    trama[0] = RMD_CMD_VELOCIDAD;
    trama[1] = trama[2] = trama[3] = 0;
    trama[4] = (uint8_t)(v & 0xFF);
    trama[5] = (uint8_t)((v >> 8) & 0xFF);
    trama[6] = (uint8_t)((v >> 16) & 0xFF);
    trama[7] = (uint8_t)((v >> 24) & 0xFF);

    unsigned long fin = millis() + duracion[f];
    while (millis() < fin) {
      vaciarRx();
      unsigned long t = millis();
      uint8_t r[8];
      if (enviarTrama(ID_PRIMERO, trama) &&
          esperarRespuesta(ID_PRIMERO, r, 10) > 0) {
        Serial.print(F("DAT,"));
        Serial.print(t - t0);       Serial.print(F(","));
        Serial.print(fases[f], 0);  Serial.print(F(","));
        Serial.print(i16(r + 4));   Serial.print(F(","));
        Serial.print(i16(r + 2));   Serial.print(F(","));
        Serial.print((int8_t)r[1]); Serial.print(F(","));
        Serial.println(i16(r + 6));
      } else {
        Serial.print(F("MISS,"));
        Serial.print(t - t0);       Serial.print(F(","));
        Serial.println(fases[f], 0);
      }
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        apagarTodos();
        Serial.println(F("CORTADO. Motor apagado."));
        return;
      }
      delay(18);
    }
  }

  apagarTodos();
  Serial.println(F("FIN. Motor apagado."));
  Serial.println();
}

/* El TWAI si sabe recuperarse de un bus-off, al contrario que el MCP2515
 * con la libreria CAN (alli hay que resetear la placa entera). */
void recuperarBus() {
  twai_status_info_t st;
  twai_get_status_info(&st);
  if (st.state != TWAI_STATE_BUS_OFF) {
    Serial.println(F("El bus no esta caido; no hay nada que recuperar."));
    return;
  }
  Serial.print(F("Iniciando recuperacion... "));
  if (recuperarDeBusOff()) {
    Serial.println(F("OK. Comprueba con 'i'."));
  } else {
    Serial.println(F("no se pudo. Reinicia la placa."));
  }
}

/* Lee el angulo absoluto multivuelta (0.01 grados). Sobrevive al apagon,
 * asi que sirve para medir cuanto se mueve el eje en la sacudida. */
bool leerAnguloAbs(float *grados) {
  vaciarRx();
  uint8_t pet[8] = {RMD_CMD_ANG_MULTI, 0, 0, 0, 0, 0, 0, 0};
  uint8_t r[8];
  if (!enviarTrama(ID_PRIMERO, pet) ||
      esperarRespuesta(ID_PRIMERO, r, 60) == 0) return false;
  if (r[0] != RMD_CMD_ANG_MULTI) return false;
  int32_t v = (int32_t)((uint32_t)r[4] | ((uint32_t)r[5] << 8) |
                        ((uint32_t)r[6] << 16) | ((uint32_t)r[7] << 24));
  *grados = v / 100.0f;
  return true;
}

/* Observa un ciclo de apagado y encendido del motor.
 * La gracia frente al MKR: si el bus se cae mientras el motor esta sin
 * alimentar, el TWAI lo detecta y se recupera SOLO, sin resetear la placa. */
void observarReinicio() {
  Serial.println();
  Serial.println(F("=== CICLO DE APAGADO Y ENCENDIDO ==="));

  float angAntes = 0.0f;
  if (leerAnguloAbs(&angAntes)) {
    Serial.print(F("Angulo absoluto ANTES: "));
    Serial.print(angAntes, 2);
    Serial.println(F(" grados"));
  } else {
    Serial.println(F("No se pudo leer el angulo (el motor no responde)."));
  }

  Serial.println();
  Serial.println(F("AHORA: apaga el motor, espera unos segundos y enciendelo."));
  Serial.println(F("Yo sigo escuchando y me recupero solo del bus-off."));
  Serial.println(F("Pulsa Enter cuando quieras terminar (60 s maximo)."));
  Serial.println(F("CSV: DAT,ms,vel,corr,temp,pos | EV,ms,evento"));
  Serial.println(F("INICIO"));

  unsigned long t0 = millis();
  unsigned long tUltimaResp = millis();
  int recuperaciones = 0;
  bool huboHueco = false;

  while (millis() - t0 < 60000UL) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }

    /* Si el bus se cae por estar el motor sin alimentar, recuperarlo */
    twai_status_info_t st;
    twai_get_status_info(&st);
    if (st.state == TWAI_STATE_BUS_OFF) {
      Serial.print(F("EV,")); Serial.print(millis() - t0);
      Serial.println(F(",BUS-OFF detectado, recuperando"));
      bool ok = recuperarDeBusOff();
      recuperaciones++;
      Serial.print(F("EV,")); Serial.print(millis() - t0);
      Serial.println(ok ? F(",bus recuperado") : F(",recuperacion FALLIDA"));
      huboHueco = true;
      continue;
    }

    vaciarRx();
    unsigned long t = millis();
    uint8_t pet[8] = {RMD_CMD_ESTADO2, 0, 0, 0, 0, 0, 0, 0};
    uint8_t r[8];
    if (enviarTrama(ID_PRIMERO, pet, 5) &&
        esperarRespuesta(ID_PRIMERO, r, 10) > 0) {
      if (huboHueco || t - tUltimaResp > 500) {
        /* primer contacto tras el apagon: leer flags y tension del arranque */
        vaciarRx();
        uint8_t p1[8] = {RMD_CMD_ESTADO1, 0, 0, 0, 0, 0, 0, 0};
        uint8_t rr[8];
        if (enviarTrama(ID_PRIMERO, p1) &&
            esperarRespuesta(ID_PRIMERO, rr, 30) > 0) {
          uint16_t dv = (uint16_t)rr[4] | ((uint16_t)rr[5] << 8);
          uint16_t fl = (uint16_t)rr[6] | ((uint16_t)rr[7] << 8);
          Serial.print(F("EV,")); Serial.print(t - t0);
          Serial.print(F(",ARRANQUE flags=0x")); Serial.print(fl, HEX);
          Serial.print(F(" tension=")); Serial.print(dv / 10.0f, 1);
          Serial.println(F("V"));
        }
        huboHueco = false;
      }
      Serial.print(F("DAT,"));
      Serial.print(t - t0);        Serial.print(F(","));
      Serial.print(i16(r + 4));    Serial.print(F(","));
      Serial.print(i16(r + 2));    Serial.print(F(","));
      Serial.print((int8_t)r[1]);  Serial.print(F(","));
      Serial.println(i16(r + 6));
      tUltimaResp = t;
    } else {
      Serial.print(F("MISS,"));
      Serial.println(t - t0);
      if (millis() - tUltimaResp > 500) huboHueco = true;
    }
    delay(14);
  }

  Serial.println(F("FIN OBSERVACION"));
  Serial.print(F("Recuperaciones de bus-off: "));
  Serial.println(recuperaciones);

  float angDespues = 0.0f;
  if (leerAnguloAbs(&angDespues)) {
    Serial.print(F("Angulo absoluto DESPUES: "));
    Serial.print(angDespues, 2);
    Serial.println(F(" grados"));
    Serial.print(F(">>> DESPLAZAMIENTO DE LA SACUDIDA: "));
    Serial.print(angDespues - angAntes, 2);
    Serial.println(F(" grados"));
  }
  Serial.println();
}

/* ---- ARRANQUE --------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(400);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - CAN del ESP32 (TWAI) a 1 Mbps"));
  Serial.println(F("=================================================="));
  Serial.print(F("  TX: GPIO ")); Serial.print((int)PIN_TWAI_TX);
  Serial.print(F("   RX: GPIO ")); Serial.println((int)PIN_TWAI_RX);
  Serial.println();

  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.print(F("Instalando el driver TWAI... "));
  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println(F("FALLO"));
    Serial.println(F("Revisa que los pines no esten usados por otra cosa."));
    return;
  }
  Serial.println(F("OK"));

  Serial.print(F("Arrancando el bus... "));
  if (twai_start() != ESP_OK) {
    Serial.println(F("FALLO"));
    return;
  }
  Serial.println(F("OK"));
  twaiListo = true;

  Serial.println();
  Serial.println(F("OJO: el driver arranca aunque el transceptor este mal"));
  Serial.println(F("cableado. La prueba de verdad es 's' y luego 'i'."));

  ayuda();
  Serial.println(F("Empieza por 's': escanear IDs."));
  Serial.println();
}

void loop() {
  if (!twaiListo || !Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  switch (linea.charAt(0)) {
    case 's': case 'S': escanearIDs(); break;
    case 'i': case 'I': estadoBus(); break;
    case 't': case 'T': pruebaCarga(); break;
    case 'm': case 'M': moverMotor(); break;
    case 'w': case 'W': observarReinicio(); break;
    case 'l': case 'L': apagarTodos(); Serial.println(F("Motores apagados.")); break;
    case 'r': case 'R': recuperarBus(); break;
    case 'h': case 'H': ayuda(); break;
    default: Serial.println(F("No entiendo. Escribe 'h'.")); break;
  }
}
