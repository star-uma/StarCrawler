/*
 * test_can_mkr.ino - StarCrawler - PRUEBA 2 de 4
 * ================================================================
 *
 *   QUE PRUEBA ESTO
 *   Los 4 motores de traccion (RMD-X8) que van por el bus CAN.
 *   Sirve para dos cosas:
 *     1. Saber que ID tiene de verdad cada motor
 *     2. Probar los motores DE UNO EN UNO, para aislar cual falla
 *
 *   POR QUE SOBRE EL MKR Y NO SOBRE EL ESP32
 *   El MKR con su shield CAN ya funciona y ya ha hablado con estos
 *   motores a 1 Mbps. El modulo CAN del ESP32 todavia no esta
 *   montado. Para diagnosticar motores interesa usar hardware que
 *   ya sabemos que funciona: asi, si algo falla, sabemos que es el
 *   motor o el cableado, y no la electronica nueva sin verificar.
 *
 *   PLACA: Arduino MKR WiFi 1010 + MKR CAN Shield
 *
 *   ================================================================
 *   ANTES DE EMPEZAR - LEE ESTO
 *   ================================================================
 *
 *   1. EL ROBOT TIENE QUE ESTAR SOBRE TACOS (con las orugas al aire)
 *      antes de usar cualquier comando que mueva motores.
 *
 *   2. NO puede haber dos placas mandando en el bus CAN a la vez.
 *      Si el ESP32 esta conectado al bus, desconectalo o no lo
 *      alimentes. Dos placas mandando ordenes a los mismos motores
 *      producen justo el tipo de comportamiento raro que estamos
 *      intentando diagnosticar.
 *
 *   3. Ten localizado el interruptor de alimentacion de los motores.
 *
 *   ================================================================
 *   ORDEN RECOMENDADO
 *   ================================================================
 *
 *     Paso 1 -> comando 'e'  (escanear IDs). NO mueve nada. Riesgo cero.
 *     Paso 2 -> comando 'd'  (prueba del retardo entre tramas).
 *     Paso 3 -> comando 'm'  (mover un motor). YA MUEVE: robot sobre tacos.
 *
 *   ================================================================
 *   QUE TIENES QUE APUNTAR
 *   ================================================================
 *
 *     ID     | responde | rueda que se mueve | sentido | anomalias
 *     -------+----------+--------------------+---------+----------
 *     0x141  |          |                    |         |
 *     0x142  |          |                    |         |
 *     0x143  |          |                    |         |
 *     0x144  |          |                    |         |
 *
 * ================================================================
 */

#include <CAN.h>

/* ---- CONFIGURACION --------------------------------------------
 * Valores sacados de firmware/starcrawler_esp32/config.h
 */
#define VELOCIDAD_BUS     1000E3   /* 1 Mbps: lo que usan los RMD-X8 */
#define CAN_INTER_FRAME_US  250    /* retardo entre tramas. Ver comando 'd' */

/* IDs que el firmware da por supuestos. Justamente vamos a
 * comprobar si son estos de verdad. */
#define ID_FL  0x141
#define ID_FR  0x142
#define ID_RR  0x143
#define ID_RL  0x144

/* Rango que barre el comando de escaneo */
#define ID_PRIMERO  0x141
#define ID_ULTIMO   0x148

/* Parametros de la prueba de movimiento (conservadores a proposito) */
#define VEL_PRUEBA_DPS      5.0f   /* muy lento: 5 grados por segundo */
#define DURACION_PRUEBA_MS  2000   /* 2 segundos y para solo */

/* Comandos del protocolo RMD-X8 que usa el firmware del proyecto */
#define RMD_CMD_VELOCIDAD  0xA2   /* control de velocidad en lazo cerrado */
#define RMD_CMD_APAGAR     0x80   /* apaga el motor: queda suelto, sin par */
#define RMD_CMD_ESTADO     0x9C   /* lectura de estado (no mueve nada) */

bool canListo = false;

/* ---- ENVIO Y RECEPCION ----------------------------------------- */

/* Manda una trama de 8 bytes al ID indicado. */
bool enviarTrama(long id, const uint8_t datos[8]) {
  if (!CAN.beginPacket(id)) return false;
  for (int i = 0; i < 8; i++) CAN.write(datos[i]);
  return CAN.endPacket() != 0;
}

/* Prepara la trama de "gira a esta velocidad".
 * La velocidad va en centesimas de grado por segundo. */
void tramaVelocidad(float dps, uint8_t out[8]) {
  int32_t v = (int32_t)lround(dps * 100.0);
  out[0] = RMD_CMD_VELOCIDAD;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = 0x00;
  out[4] = (uint8_t)( v        & 0xFF);
  out[5] = (uint8_t)((v >>  8) & 0xFF);
  out[6] = (uint8_t)((v >> 16) & 0xFF);
  out[7] = (uint8_t)((v >> 24) & 0xFF);
}

/* Prepara la trama de "apaga el motor" (queda suelto, sin par). */
void tramaApagar(uint8_t out[8]) {
  for (int i = 0; i < 8; i++) out[i] = 0x00;
  out[0] = RMD_CMD_APAGAR;
}

/* Espera una respuesta durante un tiempo limite.
 * Devuelve el numero de bytes recibidos, o 0 si no contesto nadie. */
int esperarRespuesta(long *idRecibido, uint8_t datos[8], unsigned long msLimite) {
  unsigned long limite = millis() + msLimite;
  while (millis() < limite) {
    int tam = CAN.parsePacket();
    if (tam > 0) {
      *idRecibido = CAN.packetId();
      int n = 0;
      while (CAN.available() && n < 8) datos[n++] = CAN.read();
      return n;
    }
  }
  return 0;
}

/* Apaga los 4 motores. Se llama siempre al terminar cualquier prueba
 * y tambien con el comando de parada de emergencia. */
void apagarTodos() {
  uint8_t t[8];
  tramaApagar(t);
  for (long id = ID_PRIMERO; id <= ID_ULTIMO; id++) {
    enviarTrama(id, t);
    delayMicroseconds(CAN_INTER_FRAME_US);
  }
}

/* Lee una linea del puerto serie, en mayusculas. */
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
  Serial.println(F("|  SIN MOVIMIENTO (riesgo cero)                        |"));
  Serial.println(F("|    e   Escanear IDs: quien responde en el bus        |"));
  Serial.println(F("|    l   Apagar todos los motores (quedan sueltos)     |"));
  Serial.println(F("|    d   Prueba del retardo de 250 us entre tramas     |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|  CON MOVIMIENTO (robot SOBRE TACOS)                  |"));
  Serial.println(F("|    m   Mover UN motor, muy despacio, 2 segundos      |"));
  Serial.println(F("|                                                      |"));
  Serial.println(F("|    p   PARADA DE EMERGENCIA (apaga todo)             |"));
  Serial.println(F("|    h   Mostrar esta ayuda                            |"));
  Serial.println(F("+------------------------------------------------------+"));
  Serial.println();
}

void escanearIDs() {
  Serial.println();
  Serial.println(F("=== ESCANEO DE IDs EN EL BUS CAN ==="));
  Serial.println();
  Serial.println(F("Se pregunta el estado a cada ID. Esto NO mueve nada:"));
  Serial.println(F("es solo una lectura."));
  Serial.println();

  int responden = 0;
  for (long id = ID_PRIMERO; id <= ID_ULTIMO; id++) {
    /* Vaciamos lo que hubiera pendiente de antes */
    while (CAN.parsePacket() > 0) { while (CAN.available()) CAN.read(); }

    uint8_t peticion[8] = {RMD_CMD_ESTADO, 0, 0, 0, 0, 0, 0, 0};

    Serial.print(F("  0x"));
    Serial.print(id, HEX);
    Serial.print(F("  ->  "));

    if (!enviarTrama(id, peticion)) {
      Serial.println(F("no se pudo ni enviar (bus saturado o mal conectado)"));
      delay(50);
      continue;
    }

    long idResp;
    uint8_t datos[8];
    int n = esperarRespuesta(&idResp, datos, 60);

    if (n == 0) {
      Serial.println(F("SIN RESPUESTA"));
    } else {
      Serial.print(F("RESPONDE"));
      if (idResp != id) {
        Serial.print(F("  [OJO: contesta con ID 0x"));
        Serial.print(idResp, HEX);
        Serial.print(F("]"));
      }
      Serial.print(F("   bytes:"));
      for (int i = 0; i < n; i++) {
        Serial.print(F(" "));
        if (datos[i] < 0x10) Serial.print(F("0"));
        Serial.print(datos[i], HEX);
      }
      Serial.println();
      responden++;

      /* Si contesta mas de uno al mismo ID hay IDs duplicados */
      long idExtra;
      uint8_t extra[8];
      if (esperarRespuesta(&idExtra, extra, 20) > 0) {
        Serial.println(F("          *** AVISO: contesta MAS DE UN dispositivo"));
        Serial.println(F("          a este ID. Probablemente hay dos motores"));
        Serial.println(F("          con el mismo ID configurado. Eso explica"));
        Serial.println(F("          comportamientos raros."));
      }
    }
    delay(80);
  }

  Serial.println();
  Serial.print(F("Resumen: responden "));
  Serial.print(responden);
  Serial.println(F(" dispositivos."));
  Serial.println();
  Serial.println(F("Lo esperado son 4: 0x141, 0x142, 0x143 y 0x144."));
  if (responden == 0) {
    Serial.println();
    Serial.println(F("NINGUNO responde. Antes de sospechar de los motores,"));
    Serial.println(F("comprueba en este orden:"));
    Serial.println(F("  1. Que los motores esten alimentados"));
    Serial.println(F("  2. Las resistencias de terminacion: 120 ohm SOLO en"));
    Serial.println(F("     los dos extremos del bus, no en cada motor"));
    Serial.println(F("  3. Que CAN-H y CAN-L no esten cruzados"));
    Serial.println(F("  4. Que el shield CAN este bien encajado"));
  } else if (responden < 4) {
    Serial.println();
    Serial.println(F("Faltan motores. Apunta cuales y sigue con los que si"));
    Serial.println(F("responden: el comando 'm' te dira que rueda es cada uno."));
  }
  Serial.println();
}

void pruebaRetardo() {
  Serial.println();
  Serial.println(F("=== PRUEBA DEL RETARDO ENTRE TRAMAS ==="));
  Serial.println();
  Serial.println(F("CONTEXTO"));
  Serial.println(F("En el sistema original habia un fallo conocido con el"));
  Serial.println(F("motor FL (0x141): si las tramas se mandaban demasiado"));
  Serial.println(F("seguidas, ese motor hacia cosas raras. Se arreglo"));
  Serial.println(F("metiendo 250 microsegundos entre trama y trama."));
  Serial.println();
  Serial.println(F("Esta prueba manda lecturas de estado a los 4 IDs, primero"));
  Serial.println(F("SIN retardo y luego CON retardo, y cuenta cuantas"));
  Serial.println(F("respuestas se pierden en cada caso. NO mueve motores."));
  Serial.println();
  Serial.println(F("Si sin retardo se pierden respuestas y con retardo no,"));
  Serial.println(F("ese es exactamente el fallo conocido: el motor esta bien."));
  Serial.println();

  const long ids[4] = {ID_FL, ID_FR, ID_RR, ID_RL};

  for (int fase = 0; fase < 2; fase++) {
    bool conRetardo = (fase == 1);
    Serial.print(F("--- "));
    Serial.print(conRetardo ? F("CON retardo de 250 us") : F("SIN retardo"));
    Serial.println(F(" (30 rondas) ---"));

    int enviadas = 0, recibidas = 0;
    for (int ronda = 0; ronda < 30; ronda++) {
      for (int k = 0; k < 4; k++) {
        while (CAN.parsePacket() > 0) { while (CAN.available()) CAN.read(); }

        uint8_t peticion[8] = {RMD_CMD_ESTADO, 0, 0, 0, 0, 0, 0, 0};
        if (enviarTrama(ids[k], peticion)) enviadas++;

        long idResp;
        uint8_t datos[8];
        if (esperarRespuesta(&idResp, datos, 15) > 0) recibidas++;

        if (conRetardo) delayMicroseconds(CAN_INTER_FRAME_US);
      }
    }

    Serial.print(F("    enviadas "));
    Serial.print(enviadas);
    Serial.print(F(" / respondidas "));
    Serial.print(recibidas);
    Serial.print(F("   -> perdidas: "));
    Serial.println(enviadas - recibidas);
    Serial.println();
    delay(300);
  }

  Serial.println(F("COMO INTERPRETARLO"));
  Serial.println(F("  - Si pierde bastantes SIN retardo y casi ninguna CON"));
  Serial.println(F("    retardo: es el fallo conocido. Deja siempre el retardo."));
  Serial.println(F("  - Si pierde en los dos casos por igual: el problema es"));
  Serial.println(F("    del bus (terminacion, cables, cristal del modulo)."));
  Serial.println(F("  - Si no pierde ninguna: el bus esta fino."));
  Serial.println();
}

void moverUnMotor() {
  Serial.println();
  Serial.println(F("=== MOVER UN MOTOR ==="));
  Serial.println();
  Serial.println(F("*** ESTE COMANDO MUEVE UN MOTOR DE VERDAD ***"));
  Serial.println();
  Serial.println(F("Comprueba antes de seguir:"));
  Serial.println(F("  [ ] El robot esta SOBRE TACOS, con las orugas al aire"));
  Serial.println(F("  [ ] No hay nadie ni nada tocando las orugas"));
  Serial.println(F("  [ ] Sabes donde esta el interruptor de los motores"));
  Serial.println();
  Serial.println(F("Se movera UN SOLO motor, a 5 grados/segundo (muy"));
  Serial.println(F("despacio), durante 2 segundos, y parara solo."));
  Serial.println(F("Puedes cortar antes pulsando Enter."));
  Serial.println();
  Serial.println(F("Que ID quieres mover?"));
  Serial.println(F("   1 -> 0x141      3 -> 0x143"));
  Serial.println(F("   2 -> 0x142      4 -> 0x144"));
  Serial.println(F("   x -> cancelar"));

  String eleccion = leerLinea(60000UL);
  long id = 0;
  if      (eleccion == "1") id = 0x141;
  else if (eleccion == "2") id = 0x142;
  else if (eleccion == "3") id = 0x143;
  else if (eleccion == "4") id = 0x144;
  else {
    Serial.println(F("Cancelado. No se ha movido nada."));
    Serial.println();
    return;
  }

  Serial.println();
  Serial.print(F("Vas a mover el motor 0x"));
  Serial.println(id, HEX);
  Serial.println(F("Escribe SI y pulsa Enter para confirmar."));
  Serial.println(F("(cualquier otra cosa cancela)"));

  String confirma = leerLinea(60000UL);
  if (confirma != "SI") {
    Serial.println(F("Cancelado. No se ha movido nada."));
    Serial.println();
    return;
  }

  Serial.println();
  Serial.println(F(">>> MIRA EL ROBOT. Apunta QUE RUEDA se mueve y HACIA"));
  Serial.println(F(">>> DONDE gira (adelante o atras)."));
  Serial.println();
  delay(1500);
  Serial.println(F("Moviendo..."));

  uint8_t trama[8];
  tramaVelocidad(VEL_PRUEBA_DPS, trama);

  unsigned long fin = millis() + DURACION_PRUEBA_MS;
  bool cortado = false;
  while (millis() < fin) {
    enviarTrama(id, trama);
    delayMicroseconds(CAN_INTER_FRAME_US);
    delay(20);                      /* refresco a ~50 Hz */
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      cortado = true;
      break;
    }
  }

  /* Pare como pare, siempre se apaga */
  apagarTodos();

  Serial.println();
  if (cortado) Serial.println(F("Cortado por el usuario. Motor apagado."));
  else         Serial.println(F("Terminado. Motor apagado."));
  Serial.println();
  Serial.println(F("APUNTA EL RESULTADO:"));
  Serial.print(F("  ID 0x"));
  Serial.print(id, HEX);
  Serial.println(F("  ->  rueda: ______   sentido: ______"));
  Serial.println();
  Serial.println(F("Si NO se ha movido nada, prueba el comando 'e' para ver"));
  Serial.println(F("si ese ID responde siquiera."));
  Serial.println();
}

/* ---- ARRANQUE -------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) { }   /* espera al monitor serie */
  delay(500);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  StarCrawler - PRUEBA 2 de 4: MOTORES DE TRACCION"));
  Serial.println(F("  (Arduino MKR WiFi 1010 + shield CAN)"));
  Serial.println(F("=================================================="));
  Serial.println();

  Serial.print(F("Arrancando el bus CAN a 1 Mbps... "));
  if (CAN.begin(VELOCIDAD_BUS)) {
    canListo = true;
    Serial.println(F("OK"));
  } else {
    canListo = false;
    Serial.println(F("FALLO"));
    Serial.println();
    Serial.println(F("No se ha podido inicializar el controlador CAN."));
    Serial.println(F("Revisa:"));
    Serial.println(F("  - Que el shield CAN este bien encajado en el MKR"));
    Serial.println(F("  - Que el shield tenga alimentacion"));
    Serial.println(F("  - Que no haya otro programa usando la placa"));
    Serial.println();
    Serial.println(F("Sin esto no se puede hacer nada mas."));
    return;
  }

  /* Arranque en estado seguro: todos los motores apagados */
  apagarTodos();
  Serial.println(F("Motores apagados (estado seguro de arranque)."));

  Serial.println();
  Serial.println(F("RECUERDA:"));
  Serial.println(F("  - Robot SOBRE TACOS antes de mover nada"));
  Serial.println(F("  - El ESP32 NO puede estar mandando en el bus a la vez"));

  ayuda();
  Serial.println(F("Empieza por 'e': escanear IDs. No mueve nada."));
  Serial.println();
}

void loop() {
  if (!canListo) return;
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;
  char c = linea.charAt(0);

  switch (c) {
    case 'e': case 'E':
      escanearIDs();
      break;

    case 'l': case 'L':
      apagarTodos();
      Serial.println(F(">>> Los 4 motores apagados (sueltos, sin par)."));
      Serial.println();
      break;

    case 'd': case 'D':
      pruebaRetardo();
      break;

    case 'm': case 'M':
      moverUnMotor();
      break;

    case 'p': case 'P':
      apagarTodos();
      Serial.println();
      Serial.println(F("*** PARADA DE EMERGENCIA: todo apagado ***"));
      Serial.println();
      break;

    case 'h': case 'H': case '?':
      ayuda();
      break;

    default:
      Serial.println(F("No entiendo esa orden. Escribe 'h' para la ayuda."));
      break;
  }
}
