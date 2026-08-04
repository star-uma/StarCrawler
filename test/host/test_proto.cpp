/*
 * test_proto.cpp
 * ==============
 * Tests del protocolo serie ESP32 <-> PC (ROS 2).
 *
 * El caso clave es test_trama_canonica(): comprueba que la trama generada en C
 * es EXACTAMENTE la misma que genera protocol.py en el PC
 * (ros2_ws/src/starcrawler_driver/test/test_protocol.py comparte el mismo
 * literal hexadecimal). Si alguien cambia un campo en un lado, falla aqui.
 */
#include <cstdio>
#include <cstring>
#include <string>

#include "../../firmware/starcrawler_esp32_ros2/proto.h"

static int pruebas = 0;
static int fallos = 0;

#define CHECK(cond, msg)                                    \
  do {                                                      \
    pruebas++;                                              \
    if (!(cond)) {                                          \
      fallos++;                                             \
      printf("  [FALLO] %s (linea %d)\n", msg, __LINE__);    \
    }                                                       \
  } while (0)

static std::string hex(const uint8_t *d, int n) {
  static const char *D = "0123456789abcdef";
  std::string s;
  for (int i = 0; i < n; i++) {
    s += D[d[i] >> 4];
    s += D[d[i] & 0xF];
  }
  return s;
}

/* ── CRC ────────────────────────────────────────────────────────────────── */

static void testCrc() {
  printf("crc16 (CCITT-FALSE):\n");
  const char *v = "123456789";
  CHECK(sp_crc16((const uint8_t *)v, 9) == 0x29B1, "vector estandar 0x29B1");
  CHECK(sp_crc16((const uint8_t *)"", 0) == 0xFFFF, "cadena vacia -> 0xFFFF");
}

/* ── Trama canonica: contrato con protocol.py ───────────────────────────── */

static void testTramaCanonica() {
  printf("trama canonica de comando (contrato C <-> Python):\n");
  sp_Comando cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.vel_izq_cdps = 1234;
  cmd.vel_der_cdps = -1234;
  cmd.incremento[0] = 1;
  cmd.incremento[1] = -1;
  cmd.incremento[2] = 0;
  cmd.incremento[3] = 1;
  cmd.objetivo_cdeg[0] = 22500;
  cmd.objetivo_cdeg[1] = 13500;
  cmd.objetivo_cdeg[2] = 13500;
  cmd.objetivo_cdeg[3] = 22500;
  cmd.flags = SP_FLAG_CMD_USAR_POSICION;
  cmd.seq = 7;

  uint8_t trama[SP_TAM_TRAMA_CMD];
  const int n = sp_empaquetarComando(&cmd, trama);
  CHECK(n == SP_TAM_TRAMA_CMD && n == 26, "longitud 26 bytes");

  /* Mismo literal que test_protocol.py::test_trama_canonica_de_comando */
  const std::string esperado =
      "a55a0114"          /* SOF, tipo, len   */
      "d204"              /* vel_izq = 1234   */
      "2efb"              /* vel_der = -1234  */
      "01ff0001"          /* incrementos      */
      "e457bc34bc34e457"  /* objetivos        */
      "01"                /* flags            */
      "07"                /* seq              */
      "0000"              /* reservado        */
      "dd57";             /* crc16            */
  const std::string obtenido = hex(trama, n);
  CHECK(obtenido == esperado, "coincide byte a byte con protocol.py");
  if (obtenido != esperado) {
    printf("        esperado: %s\n", esperado.c_str());
    printf("        obtenido: %s\n", obtenido.c_str());
  }
}

/* ── Ida y vuelta ───────────────────────────────────────────────────────── */

static void testIdaYVueltaComando() {
  printf("ida y vuelta de comando:\n");
  sp_Comando a;
  memset(&a, 0, sizeof(a));
  a.vel_izq_cdps = -4000;
  a.vel_der_cdps = 4000;
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    a.incremento[i] = (int8_t)((i % 2) ? 1 : -1);
    a.objetivo_cdeg[i] = (int16_t)(9000 + 4500 * i);
  }
  a.flags = SP_FLAG_CMD_EMERGENCIA;
  a.seq = 250;

  uint8_t trama[SP_TAM_TRAMA_CMD];
  sp_empaquetarComando(&a, trama);
  sp_Comando b;
  memset(&b, 0, sizeof(b));
  sp_desempaquetarComando(trama + 4, &b);

  CHECK(b.vel_izq_cdps == a.vel_izq_cdps && b.vel_der_cdps == a.vel_der_cdps,
        "velocidades con signo");
  bool ok = true;
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    ok &= (b.incremento[i] == a.incremento[i]);
    ok &= (b.objetivo_cdeg[i] == a.objetivo_cdeg[i]);
  }
  CHECK(ok, "incrementos y objetivos");
  CHECK(b.flags == a.flags && b.seq == a.seq, "flags y seq");
}

static void testIdaYVueltaEstado() {
  printf("ida y vuelta de estado:\n");
  sp_Estado a;
  memset(&a, 0, sizeof(a));
  a.angulo_cdeg[0] = 18000;
  a.angulo_cdeg[1] = 9012;
  a.angulo_cdeg[2] = 27099;
  a.angulo_cdeg[3] = 13500;
  a.vel_izq_cdps = -1500;
  a.vel_der_cdps = 1500;
  a.error_bits = 0x0045;
  a.flags = SP_FLAG_ST_SEGURIDAD;
  a.seq_eco = 42;

  uint8_t trama[SP_TAM_TRAMA_STATE];
  const int n = sp_empaquetarEstado(&a, trama);
  CHECK(n == 26, "longitud 26 bytes");
  CHECK(trama[2] == SP_TIPO_STATE, "tipo STATE");

  sp_Estado b;
  memset(&b, 0, sizeof(b));
  sp_desempaquetarEstado(trama + 4, &b);
  bool ok = true;
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    ok &= (b.angulo_cdeg[i] == a.angulo_cdeg[i]);
  }
  CHECK(ok, "angulos");
  CHECK(b.error_bits == a.error_bits && b.flags == a.flags, "errores y flags");
  CHECK(b.seq_eco == a.seq_eco, "eco de seq");
}

/* ── Parser ─────────────────────────────────────────────────────────────── */

static int alimentar(sp_Parser *p, const uint8_t *d, int n, uint8_t *payload) {
  int completas = 0;
  for (int i = 0; i < n; i++) {
    if (sp_parserByte(p, d[i], payload)) completas++;
  }
  return completas;
}

static void tramaCmdSeq(uint8_t seq, uint8_t *out) {
  sp_Comando c;
  memset(&c, 0, sizeof(c));
  c.seq = seq;
  sp_empaquetarComando(&c, out);
}

static void testParser() {
  printf("parser incremental:\n");
  uint8_t payload[SP_LEN_CMD];
  uint8_t t1[SP_TAM_TRAMA_CMD], t2[SP_TAM_TRAMA_CMD];
  tramaCmdSeq(1, t1);
  tramaCmdSeq(2, t2);

  /* Dos tramas seguidas */
  sp_Parser p;
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  uint8_t juntas[2 * SP_TAM_TRAMA_CMD];
  memcpy(juntas, t1, SP_TAM_TRAMA_CMD);
  memcpy(juntas + SP_TAM_TRAMA_CMD, t2, SP_TAM_TRAMA_CMD);
  CHECK(alimentar(&p, juntas, sizeof(juntas), payload) == 2,
        "extrae dos tramas seguidas");
  CHECK(p.tramas_ok == 2 && p.errores_crc == 0, "contadores correctos");

  /* Resincroniza tras basura */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  const uint8_t basura[] = {0x00, 'x', 'y', 0xFF, 0x12};
  alimentar(&p, basura, sizeof(basura), payload);
  CHECK(alimentar(&p, t1, SP_TAM_TRAMA_CMD, payload) == 1,
        "resincroniza tras basura");
  CHECK(p.bytes_descartados > 0, "cuenta los bytes descartados");

  /* Byte a byte: solo completa con el ultimo */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  bool prematuro = false;
  for (int i = 0; i < SP_TAM_TRAMA_CMD - 1; i++) {
    if (sp_parserByte(&p, t1[i], payload)) prematuro = true;
  }
  CHECK(!prematuro, "no completa antes de tiempo");
  CHECK(sp_parserByte(&p, t1[SP_TAM_TRAMA_CMD - 1], payload),
        "completa con el ultimo byte");

  /* CRC corrupto */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  uint8_t corrupta[SP_TAM_TRAMA_CMD];
  memcpy(corrupta, t1, SP_TAM_TRAMA_CMD);
  corrupta[10] ^= 0xFF;
  CHECK(alimentar(&p, corrupta, SP_TAM_TRAMA_CMD, payload) == 0,
        "descarta trama con CRC malo");
  CHECK(p.errores_crc == 1 && p.tramas_ok == 0, "cuenta el error de CRC");

  /* Ignora el otro tipo de trama (el ESP32 no debe leerse su propia
   * telemetria si algo hace eco) */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  sp_Estado st;
  memset(&st, 0, sizeof(st));
  uint8_t tramaEstado[SP_TAM_TRAMA_STATE];
  sp_empaquetarEstado(&st, tramaEstado);
  CHECK(alimentar(&p, tramaEstado, SP_TAM_TRAMA_STATE, payload) == 0,
        "ignora tramas de otro tipo");
  CHECK(alimentar(&p, t1, SP_TAM_TRAMA_CMD, payload) == 1,
        "sigue funcionando despues");

  /* SOF suelto previo */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  const uint8_t sof = SP_SOF0;
  alimentar(&p, &sof, 1, payload);
  CHECK(alimentar(&p, t1, SP_TAM_TRAMA_CMD, payload) == 1,
        "tolera un 0xA5 huerfano antes de la trama");

  /* Payload que contiene bytes de sincronismo */
  sp_parserInit(&p, SP_TIPO_CMD, SP_LEN_CMD);
  sp_Comando conSof;
  memset(&conSof, 0, sizeof(conSof));
  conSof.vel_izq_cdps = (int16_t)0x5AA5;  /* SOF dentro del payload */
  conSof.objetivo_cdeg[0] = (int16_t)0xA55A;
  uint8_t tramaSof[SP_TAM_TRAMA_CMD];
  sp_empaquetarComando(&conSof, tramaSof);
  CHECK(alimentar(&p, tramaSof, SP_TAM_TRAMA_CMD, payload) == 1,
        "no se confunde con SOF dentro del payload");
  sp_Comando leido;
  memset(&leido, 0, sizeof(leido));
  sp_desempaquetarComando(payload, &leido);
  CHECK(leido.vel_izq_cdps == (int16_t)0x5AA5, "payload intacto");

  /* Parser en el sentido contrario (lo que hace el PC) */
  sp_parserInit(&p, SP_TIPO_STATE, SP_LEN_STATE);
  CHECK(alimentar(&p, tramaEstado, SP_TAM_TRAMA_STATE, payload) == 1,
        "parser de STATE extrae la telemetria");
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main() {
  printf("=== Tests del protocolo serie ROS 2 (StarCrawler) ===\n\n");

  testCrc();
  testTramaCanonica();
  testIdaYVueltaComando();
  testIdaYVueltaEstado();
  testParser();

  printf("\n=== Resultado: %d/%d pruebas OK", pruebas - fallos, pruebas);
  if (fallos > 0) {
    printf(" — %d FALLOS ===\n", fallos);
    return 1;
  }
  printf(" ===\n");
  return 0;
}
