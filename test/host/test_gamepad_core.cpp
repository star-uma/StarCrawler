/*
 * test_gamepad_core.cpp
 * =====================
 * Tests unitarios del puerto a C de StarCrawlerXbox.py (gamepad_core),
 * usado por el firmware standalone (mando Xbox directo por Bluetooth).
 *
 * Cada caso está contrastado con la función Python original.
 * Ejecutar con: ./run_tests.ps1
 */
#include <cstdio>
#include <cstring>
#include <cmath>

#include "../../firmware/starcrawler_esp32_standalone/gamepad_core.h"

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

static const float GAIN = 30.0f;
static const float VMAX = 40.0f;

static gc_EstadoMando mandoNeutro() {
  gc_EstadoMando m;
  memset(&m, 0, sizeof(m));
  return m;
}

static void checkO(const cc_DatagramaPC *d, int a, int b, int c, int e,
                   const char *msg) {
  CHECK(d->o[0] == a && d->o[1] == b && d->o[2] == c && d->o[3] == e, msg);
}

/* ── Deadzone (aplicar_deadzone del Python) ─────────────────────────────── */

static void testDeadzone() {
  printf("deadzone:\n");
  CHECK_F(gc_deadzone(0.05f, 0.08f), 0.0f, 1e-6f, "dentro de la zona -> 0");
  CHECK_F(gc_deadzone(-0.079f, 0.08f), 0.0f, 1e-6f, "negativo dentro -> 0");
  CHECK_F(gc_deadzone(1.0f, 0.08f), 1.0f, 1e-6f, "fondo de escala -> 1");
  CHECK_F(gc_deadzone(-1.0f, 0.08f), -1.0f, 1e-6f, "fondo negativo -> -1");
  /* reescalado desde el borde: (0.54-0.08)/0.92 = 0.5 */
  CHECK_F(gc_deadzone(0.54f, 0.08f), 0.5f, 1e-4f, "reescala desde el borde");
}

/* ── Cambio de modo con RB ──────────────────────────────────────────────── */

static void testCicloModos() {
  printf("ciclo de modos con RB:\n");
  gc_EstadoControl st;
  gc_reset(&st);
  CHECK(st.modo == 1, "arranca en modo 1");

  gc_EstadoMando m = mandoNeutro();
  cc_DatagramaPC d;

  m.rb = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(st.modo == 2 && d.mode == 2, "flanco RB -> modo 2");

  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(st.modo == 2, "RB mantenido no vuelve a cambiar");

  m.rb = false;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  m.rb = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(st.modo == 3, "segundo flanco -> modo 3");

  m.rb = false;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  m.rb = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(st.modo == 4, "tercer flanco -> modo 4");

  m.rb = false;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  m.rb = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(st.modo == 1, "cuarto flanco cierra el ciclo 4 -> 1 (sin modo 5)");
}

/* ── Modo 1: tracción (calcular_traccion del Python) ────────────────────── */

static void testTraccion() {
  printf("traccion:\n");
  gc_EstadoControl st;
  gc_reset(&st);
  gc_EstadoMando m = mandoNeutro();
  cc_DatagramaPC d;

  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(d.id == 1, "datagrama con id=1 (como el PC)");
  CHECK(d.mode == 1, "modo 1");
  CHECK(d.action_left_train == 0 && d.action_right_train == 0,
        "mando en reposo -> velocidad 0");

  m.ejeIzqY = 1.0f; /* todo adelante */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(d.action_left_train == 3000 && d.action_right_train == 3000,
        "adelante: 30 dps x100 en ambos trenes");

  m.ejeIzqY = 0.0f;
  m.ejeDerX = 1.0f; /* girar sobre si mismo a la derecha */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(d.action_left_train == -3000 && d.action_right_train == 3000,
        "giro puro: trenes opuestos");

  m.ejeIzqY = 1.0f; /* adelante + giro: satura el tren exterior */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(d.action_left_train == 0, "interior: 30-30 = 0");
  CHECK(d.action_right_train == 4000, "exterior: 30+30 saturado a 40 dps");

  m.ejeIzqY = -1.0f;
  m.ejeDerX = 0.0f;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  CHECK(d.action_left_train == -3000 && d.action_right_train == -3000,
        "marcha atras");
  checkO(&d, 0, 0, 0, 0, "en modo 1 el vector o[] va a cero");
}

/* ── Modo 2: posición absoluta (calcular_modo_2) ────────────────────────── */

static void testModo2() {
  printf("modo 2 (posicion absoluta):\n");
  gc_EstadoControl st;
  gc_reset(&st);
  st.modo = 2;
  gc_EstadoMando m = mandoNeutro();
  cc_DatagramaPC d;

  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 180, 180, 180, 180, "sin boton: objetivo inicial 180");
  CHECK(d.action_left_train == 0, "sin traccion en modo 2");

  m.a = true; /* A = 225 grados */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 225, 225, 225, 225, "A -> 225 en las cuatro");

  m.a = false; /* al soltar persiste la ultima consigna */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 225, 225, 225, 225, "consigna persiste al soltar");

  m.y = true; /* Y = 90 grados */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 90, 90, 90, 90, "Y -> 90 en las cuatro");
}

/* ── Modo 3: incremental x4 (calcular_modo_3) ───────────────────────────── */

static void testModo3() {
  printf("modo 3 (incremental x4):\n");
  gc_EstadoControl st;
  gc_reset(&st);
  st.modo = 3;
  gc_EstadoMando m = mandoNeutro();
  cc_DatagramaPC d;

  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 0, 0, 0, 0, "cruceta suelta -> quieto");

  m.dpadArriba = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, -1, 1, -1, 1, "arriba -> inclinar adelante");

  m = mandoNeutro();
  m.dpadAbajo = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 1, -1, 1, -1, "abajo -> inclinar atras");

  m = mandoNeutro();
  m.dpadIzq = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 1, 1, -1, -1, "izquierda -> inclinar izquierda");

  m = mandoNeutro();
  m.dpadDer = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, -1, -1, 1, 1, "derecha -> inclinar derecha");

  m.dpadArriba = true; /* diagonal: el Python exige direccion exacta */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 0, 0, 0, 0, "diagonal -> quieto (como el Python)");
}

/* ── Modo 4: incremental x2 (calcular_modo_4) ───────────────────────────── */

static void testModo4() {
  printf("modo 4 (incremental x2):\n");
  gc_EstadoControl st;
  gc_reset(&st);
  st.modo = 4;
  gc_EstadoMando m = mandoNeutro();
  cc_DatagramaPC d;

  m.dpadArriba = true; /* par seleccionado pero sin gatillo */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 0, 0, 0, 0, "sin gatillo -> quieto");

  m.trigDer = true; /* sentido = 1 (bajar) */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, -1, 1, 0, 0, "par delantero + RT");

  m = mandoNeutro();
  m.dpadAbajo = true;
  m.trigIzq = true; /* sentido = -1 (subir) */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 0, 0, -1, 1, "par trasero + LT");

  m = mandoNeutro();
  m.dpadIzq = true;
  m.trigDer = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, 0, 1, 0, -1, "par izquierdo + RT");

  m = mandoNeutro();
  m.dpadDer = true;
  m.trigDer = true;
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, -1, 0, 1, 0, "par derecho + RT");

  m.trigIzq = true; /* RT tiene prioridad, como el Python */
  gc_procesar(&st, &m, GAIN, VMAX, &d);
  checkO(&d, -1, 0, 1, 0, "ambos gatillos -> manda RT");
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main() {
  printf("=== Tests unitarios gamepad_core (StarCrawler standalone) ===\n\n");

  testDeadzone();
  testCicloModos();
  testTraccion();
  testModo2();
  testModo3();
  testModo4();

  printf("\n=== Resultado: %d/%d pruebas OK", pruebas - fallos, pruebas);
  if (fallos > 0) {
    printf(" — %d FALLOS ===\n", fallos);
    return 1;
  }
  printf(" ===\n");
  return 0;
}
