/*
 * test_control_core.cpp
 * =====================
 * Tests unitarios de la lógica pura del firmware (control_core).
 * Se ejecutan en el PC, sin hardware:
 *
 *   cd test/host
 *   ./run_tests.ps1
 *
 * Cubren: parseo de datagramas, telemetría, tramas CAN RMD, saturación,
 * rate limiter, conversión AS5600, control de posición con histéresis,
 * nivelado automático (Chart 7.4 del TFG), límites software, watchdog
 * (incluido desbordamiento de millis) y filtro complementario.
 */
#include <cstdio>
#include <cstring>
#include <cmath>

#include "../../firmware/starcrawler_esp32/control_core.h"

static int pruebas = 0;
static int fallos = 0;

#define CHECK(cond, msg)                                        \
  do {                                                          \
    pruebas++;                                                  \
    if (!(cond)) {                                              \
      fallos++;                                                 \
      printf("  [FALLO] %s (linea %d)\n", msg, __LINE__);       \
    }                                                           \
  } while (0)

#define CHECK_F(a, b, eps, msg) CHECK(std::fabs((a) - (b)) <= (eps), msg)

static void escribirInt16LE(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

/* ── Datagrama PC -> robot ──────────────────────────────────────────────── */

static void testParseDatagrama() {
  printf("parseDatagrama:\n");
  uint8_t buf[18];
  const int16_t valores[9] = {1, -1234, 4000, 225, -1, 0, 1, 2, 0};
  for (int i = 0; i < 9; i++) escribirInt16LE(buf + 2 * i, valores[i]);

  cc_DatagramaPC d;
  CHECK(cc_parseDatagrama(buf, 18, &d), "parsea 18 bytes");
  CHECK(d.id == 1, "id == 1");
  CHECK(d.action_left_train == -1234, "vel izq negativa (LE)");
  CHECK(d.action_right_train == 4000, "vel der");
  CHECK(d.o[0] == 225 && d.o[1] == -1 && d.o[2] == 0 && d.o[3] == 1,
        "vector o[] completo");
  CHECK(d.mode == 2, "modo");
  CHECK(!cc_parseDatagrama(buf, 17, &d), "rechaza paquete corto");
  CHECK(!cc_parseDatagrama(NULL, 18, &d), "rechaza buffer nulo");
}

static void testTelemetria() {
  printf("construirTelemetria:\n");
  uint8_t buf[18];
  const float ang[4] = {180.25f, 90.0f, 269.99f, 135.5f};
  int n = cc_construirTelemetria(buf, ang, -3.5f, 12.34f, 5, 0x0051);

  CHECK(n == 18, "longitud 18 bytes");
  cc_DatagramaPC d; /* mismo layout de 9 x int16: reutilizamos el parser */
  cc_parseDatagrama(buf, 18, &d);
  CHECK(d.id == 2, "origen robot (id=2)");
  CHECK(d.action_left_train == 18025, "angulo FR x100");
  CHECK(d.action_right_train == 9000, "angulo FL x100");
  CHECK(d.o[0] == 26999, "angulo RR x100");
  CHECK(d.o[1] == 13550, "angulo RL x100");
  CHECK(d.o[2] == -350, "roll x100 con signo");
  CHECK(d.o[3] == 1234, "pitch x100");
  CHECK(d.mode == 5, "modo activo");
  CHECK((uint16_t)d.code_error == 0x0051, "bits de error");
}

/* ── Tramas CAN RMD ─────────────────────────────────────────────────────── */

static void testTramasRMD() {
  printf("tramas RMD:\n");
  uint8_t t[8];

  cc_tramaVelocidadRMD(40.0f, t); /* 40 dps -> 4000 = 0x0FA0 */
  CHECK(t[0] == 0xA2, "comando 0xA2");
  CHECK(t[1] == 0 && t[2] == 0 && t[3] == 0, "bytes 1..3 a cero");
  CHECK(t[4] == 0xA0 && t[5] == 0x0F && t[6] == 0x00 && t[7] == 0x00,
        "40 dps -> int32 LE 4000");

  cc_tramaVelocidadRMD(-40.0f, t); /* -4000 = 0xFFFFF060 */
  CHECK(t[4] == 0x60 && t[5] == 0xF0 && t[6] == 0xFF && t[7] == 0xFF,
        "-40 dps -> int32 LE -4000");

  cc_tramaVelocidadRMD(0.0f, t);
  CHECK(t[4] == 0 && t[5] == 0 && t[6] == 0 && t[7] == 0, "0 dps -> cero");

  cc_tramaLiberarRMD(t);
  CHECK(t[0] == 0x80, "liberar = 0x80");
  bool restoCero = true;
  for (int i = 1; i < 8; i++) restoCero &= (t[i] == 0);
  CHECK(restoCero, "resto de la trama a cero");
}

/* ── Saturación y rate limiter ──────────────────────────────────────────── */

static void testSaturarYRate() {
  printf("saturar / rateLimiter:\n");
  CHECK_F(cc_saturar(50.0f, 40.0f), 40.0f, 1e-6f, "satura por arriba");
  CHECK_F(cc_saturar(-50.0f, 40.0f), -40.0f, 1e-6f, "satura por abajo");
  CHECK_F(cc_saturar(12.5f, 40.0f), 12.5f, 1e-6f, "pasa sin tocar");

  float v = 0.0f;
  v = cc_rateLimiter(v, 10.0f, 4.0f);
  CHECK_F(v, 4.0f, 1e-6f, "rampa paso 1");
  v = cc_rateLimiter(v, 10.0f, 4.0f);
  CHECK_F(v, 8.0f, 1e-6f, "rampa paso 2");
  v = cc_rateLimiter(v, 10.0f, 4.0f);
  CHECK_F(v, 10.0f, 1e-6f, "alcanza consigna sin sobrepasar");
  v = cc_rateLimiter(v, -10.0f, 4.0f);
  CHECK_F(v, 6.0f, 1e-6f, "deceleracion limitada");
}

/* ── Conversión AS5600 ──────────────────────────────────────────────────── */

static void testAS5600() {
  printf("as5600ADeg:\n");
  CHECK_F(cc_as5600ADeg(0, 0.0f), 0.0f, 1e-4f, "raw 0");
  CHECK_F(cc_as5600ADeg(2048, 0.0f), 180.0f, 1e-4f, "raw 2048 = 180 grados");
  CHECK_F(cc_as5600ADeg(4095, 0.0f), 359.912f, 1e-2f, "raw 4095");
  CHECK_F(cc_as5600ADeg(2048, -15.0f), 165.0f, 1e-4f, "offset RL original");
}

/* ── Control de posición con histéresis ─────────────────────────────────── */

static void testControlPosicion() {
  printf("controlPosicion:\n");
  /* umbral arranque 1.0, parada 0.5 */
  CHECK(cc_controlPosicion(170, 180, false, 1.0f, 0.5f) == 1,
        "lejos por debajo -> subir");
  CHECK(cc_controlPosicion(190, 180, false, 1.0f, 0.5f) == -1,
        "lejos por encima -> bajar");
  CHECK(cc_controlPosicion(179.5f, 180, false, 1.0f, 0.5f) == 0,
        "parado dentro del umbral de arranque -> no arranca");
  CHECK(cc_controlPosicion(179.2f, 180, true, 1.0f, 0.5f) == 1,
        "en marcha, error 0.8 > parada -> sigue");
  CHECK(cc_controlPosicion(179.7f, 180, true, 1.0f, 0.5f) == 0,
        "en marcha, error 0.3 <= parada -> para");
  CHECK(cc_controlPosicion(180, 180, true, 1.0f, 0.5f) == 0,
        "clavado en objetivo -> para");
}

/* ── Nivelado automático (Chart 7.4 del TFG, literal) ───────────────────── */

static void checkVector(const int8_t Y[4], int a, int b, int c, int d,
                        const char *msg) {
  CHECK(Y[0] == a && Y[1] == b && Y[2] == c && Y[3] == d, msg);
}

static void testNivelado() {
  printf("nivelado (Chart 7.4 TFG):\n");
  int8_t Y[4];

  cc_nivelado(0, 0, 2.0f, Y);
  checkVector(Y, 0, 0, 0, 0, "nivelado dentro del limite -> quieto");

  cc_nivelado(0, 3.0f, 2.0f, Y);
  checkVector(Y, 1, -1, 1, -1, "pitch > limite");

  cc_nivelado(0, -3.0f, 2.0f, Y);
  checkVector(Y, -1, 1, -1, 1, "pitch < -limite");

  cc_nivelado(3.0f, 0, 2.0f, Y);
  checkVector(Y, 1, 1, -1, -1, "roll > limite");

  cc_nivelado(-3.0f, 0, 2.0f, Y);
  checkVector(Y, -1, -1, 1, 1, "roll < -limite");

  /* Combinados: suma y saturación a ±1, exactamente como el Chart */
  cc_nivelado(3.0f, 3.0f, 2.0f, Y);
  checkVector(Y, 1, 0, 0, -1, "pitch+ y roll+ -> saturado {1,0,0,-1}");

  /* pitch+ {1,-1,1,-1} + roll- {-1,-1,1,1} = {0,-2,2,0} -> sat {0,-1,1,0} */
  cc_nivelado(-3.0f, 3.0f, 2.0f, Y);
  checkVector(Y, 0, -1, 1, 0, "pitch+ y roll- -> {0,-1,1,0}");

  /* pitch- {-1,1,-1,1} + roll+ {1,1,-1,-1} = {0,2,-2,0} -> sat {0,1,-1,0} */
  cc_nivelado(3.0f, -3.0f, 2.0f, Y);
  checkVector(Y, 0, 1, -1, 0, "pitch- y roll+ -> {0,1,-1,0}");

  /* Frontera: el Chart usa estrictamente '>' */
  cc_nivelado(2.0f, 2.0f, 2.0f, Y);
  checkVector(Y, 0, 0, 0, 0, "exactamente en el limite -> quieto");
}

/* ── Límites software ───────────────────────────────────────────────────── */

static void testLimites() {
  printf("aplicarLimites:\n");
  CHECK(cc_aplicarLimites(1, 276.0f, true, 85, 275) == 0,
        "bloquea subir por encima del maximo");
  CHECK(cc_aplicarLimites(-1, 276.0f, true, 85, 275) == -1,
        "permite volver hacia dentro desde arriba");
  CHECK(cc_aplicarLimites(-1, 84.0f, true, 85, 275) == 0,
        "bloquea bajar por debajo del minimo");
  CHECK(cc_aplicarLimites(1, 84.0f, true, 85, 275) == 1,
        "permite volver hacia dentro desde abajo");
  CHECK(cc_aplicarLimites(1, 180.0f, true, 85, 275) == 1,
        "en rango no interfiere");
  CHECK(cc_aplicarLimites(1, 300.0f, false, 85, 275) == 1,
        "encoder invalido -> decision del llamador");
}

/* ── Watchdog ───────────────────────────────────────────────────────────── */

static void testWatchdog() {
  printf("watchdog:\n");
  CHECK(!cc_watchdogExpirado(900, 400, 500), "500 ms exactos no expira");
  CHECK(cc_watchdogExpirado(901, 400, 500), "501 ms expira");
  /* Desbordamiento de millis(): ultimo paquete justo antes del wrap */
  const uint32_t ultimo = 0xFFFFFF9Cu; /* -100 en unsigned */
  CHECK(!cc_watchdogExpirado(100u, ultimo, 500),
        "wrap de millis: 200 ms reales no expira");
  CHECK(cc_watchdogExpirado(500u, ultimo, 500),
        "wrap de millis: 600 ms reales expira");
}

/* ── Filtro complementario ──────────────────────────────────────────────── */

static void testFiltro() {
  printf("filtro complementario:\n");
  cc_FiltroActitud f;
  cc_filtroActitudReset(&f);
  CHECK(!f.inicializado, "reset deja sin inicializar");

  /* Primera muestra siembra directo del acelerómetro */
  cc_filtroActitudUpdate(&f, 10.0f, -5.0f, 0, 0, 0.01f, 0.98f);
  CHECK_F(f.roll, 10.0f, 1e-4f, "siembra roll");
  CHECK_F(f.pitch, -5.0f, 1e-4f, "siembra pitch");

  /* Estático: converge al ángulo del acelerómetro */
  cc_filtroActitudReset(&f);
  for (int i = 0; i < 400; i++) {
    cc_filtroActitudUpdate(&f, 10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.98f);
  }
  CHECK_F(f.roll, 10.0f, 0.5f, "converge al accel en estatico");

  /* Giro puro: integra el giroscopio a corto plazo */
  cc_filtroActitudReset(&f);
  cc_filtroActitudUpdate(&f, 0, 0, 0, 0, 0.01f, 0.98f); /* siembra en 0 */
  cc_filtroActitudUpdate(&f, 0, 0, 10.0f, 0, 0.01f, 0.98f);
  CHECK_F(f.roll, 0.098f, 1e-3f, "integracion gyro un paso");
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main() {
  printf("=== Tests unitarios control_core (StarCrawler) ===\n\n");

  testParseDatagrama();
  testTelemetria();
  testTramasRMD();
  testSaturarYRate();
  testAS5600();
  testControlPosicion();
  testNivelado();
  testLimites();
  testWatchdog();
  testFiltro();

  printf("\n=== Resultado: %d/%d pruebas OK", pruebas - fallos, pruebas);
  if (fallos > 0) {
    printf(" — %d FALLOS ===\n", fallos);
    return 1;
  }
  printf(" ===\n");
  return 0;
}
