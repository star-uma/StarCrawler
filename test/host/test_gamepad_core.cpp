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

/* ── Esquema simultáneo (sin modos) ─────────────────────────────────────── */

static void checkOruga(const gc_SalidaSimultanea *s, int a, int b, int c,
                       int d, const char *msg) {
  CHECK(s->oruga[0] == a && s->oruga[1] == b && s->oruga[2] == c &&
        s->oruga[3] == d, msg);
}

static void testSimultaneoBasico() {
  printf("simultaneo - traccion y pares:\n");
  gc_EstadoSimultaneo st;
  gc_simultaneoReset(&st);
  gc_EstadoMando m = mandoNeutro();
  gc_SalidaSimultanea s;

  gc_procesarSimultaneo(&st, &m, 0, GAIN, VMAX, &s);
  CHECK(s.action_left_train == 0 && s.action_right_train == 0 &&
        !s.usarPosicion && !s.paradaEmergencia, "reposo total");

  /* Traccion y orugas A LA VEZ: la clave del esquema */
  m.ejeIzqY = 1.0f;
  m.l1 = true; /* par delantero sube */
  gc_procesarSimultaneo(&st, &m, 100, GAIN, VMAX, &s);
  CHECK(s.action_left_train == 3000 && s.action_right_train == 3000,
        "traccion activa mientras se mueve una oruga");
  checkOruga(&s, -1, 1, 0, 0, "L1: par delantero sube {-1,1,0,0}");

  m = mandoNeutro();
  m.l2 = true;
  gc_procesarSimultaneo(&st, &m, 200, GAIN, VMAX, &s);
  checkOruga(&s, 1, -1, 0, 0, "L2: par delantero baja");

  m = mandoNeutro();
  m.rb = true; /* R1 */
  gc_procesarSimultaneo(&st, &m, 300, GAIN, VMAX, &s);
  checkOruga(&s, 0, 0, 1, -1, "R1: par trasero sube");

  m = mandoNeutro();
  m.r2 = true;
  gc_procesarSimultaneo(&st, &m, 400, GAIN, VMAX, &s);
  checkOruga(&s, 0, 0, -1, 1, "R2: par trasero baja");

  m.rb = true; /* R1 + R2 a la vez se anulan */
  gc_procesarSimultaneo(&st, &m, 500, GAIN, VMAX, &s);
  checkOruga(&s, 0, 0, 0, 0, "R1+R2 simultaneos -> par quieto");

  m = mandoNeutro();
  m.dpadArriba = true;
  gc_procesarSimultaneo(&st, &m, 600, GAIN, VMAX, &s);
  checkOruga(&s, -1, 1, -1, 1, "cruceta arriba: inclinar adelante");
}

static void testSimultaneoEmergenciaYLenta() {
  printf("simultaneo - emergencia y velocidad lenta:\n");
  gc_EstadoSimultaneo st;
  gc_simultaneoReset(&st);
  gc_EstadoMando m = mandoNeutro();
  gc_SalidaSimultanea s;

  m.ejeIzqY = 1.0f;
  m.share = true;
  gc_procesarSimultaneo(&st, &m, 0, GAIN, VMAX, &s);
  CHECK(s.paradaEmergencia, "SHARE -> parada de emergencia");
  CHECK(s.action_left_train == 0 && s.oruga[0] == 0,
        "emergencia anula traccion y orugas");

  /* Toggle L3: lenta (ganancia x0.5) y vuelta */
  m = mandoNeutro();
  m.ejeIzqY = 1.0f;
  m.l3 = true;
  gc_procesarSimultaneo(&st, &m, 100, GAIN, VMAX, &s);
  CHECK(s.velocidadLenta, "flanco L3 activa velocidad lenta");
  CHECK(s.action_left_train == 1500, "lenta: 30*0.5 dps x100 = 1500");
  gc_procesarSimultaneo(&st, &m, 200, GAIN, VMAX, &s);
  CHECK(s.velocidadLenta, "L3 mantenido no vuelve a conmutar");
  m.l3 = false;
  gc_procesarSimultaneo(&st, &m, 300, GAIN, VMAX, &s);
  m.l3 = true;
  gc_procesarSimultaneo(&st, &m, 400, GAIN, VMAX, &s);
  CHECK(!s.velocidadLenta && s.action_left_train == 3000,
        "segundo flanco vuelve a rapida");
}

static void testSimultaneoPresets() {
  printf("simultaneo - presets de pose:\n");
  gc_EstadoSimultaneo st;
  gc_simultaneoReset(&st);
  gc_EstadoMando m = mandoNeutro();
  gc_SalidaSimultanea s;

  m.b = true; /* preset 180 (horizontal) */
  gc_procesarSimultaneo(&st, &m, 1000, GAIN, VMAX, &s);
  CHECK(!s.usarPosicion, "recien pulsado: aun no activa");
  gc_procesarSimultaneo(&st, &m, 1400, GAIN, VMAX, &s);
  CHECK(!s.usarPosicion, "a 400 ms sigue sin activar");
  gc_procesarSimultaneo(&st, &m, 1550, GAIN, VMAX, &s);
  CHECK(s.usarPosicion, "a 550 ms activa el preset");
  CHECK(s.objetivo[0] == 180 && s.objetivo[1] == 180 &&
        s.objetivo[2] == 180 && s.objetivo[3] == 180,
        "preset B: 180 en las cuatro");

  m.b = false; /* al soltar, el objetivo persiste */
  gc_procesarSimultaneo(&st, &m, 1700, GAIN, VMAX, &s);
  CHECK(s.usarPosicion, "objetivo persiste al soltar el boton");

  m.dpadIzq = true; /* entrada manual cancela el preset */
  gc_procesarSimultaneo(&st, &m, 1800, GAIN, VMAX, &s);
  CHECK(!s.usarPosicion, "entrada manual cancela el preset");

  /* Espejado por oruga: vertical arriba del TFG {90,270,270,90} */
  m = mandoNeutro();
  m.y = true; /* preset 90 */
  gc_procesarSimultaneo(&st, &m, 2000, GAIN, VMAX, &s);
  gc_procesarSimultaneo(&st, &m, 2600, GAIN, VMAX, &s);
  CHECK(s.objetivo[0] == 90 && s.objetivo[1] == 270 &&
        s.objetivo[2] == 270 && s.objetivo[3] == 90,
        "preset Y espejado: {90,270,270,90}");

  m = mandoNeutro();
  m.a = true; /* preset 225 */
  gc_procesarSimultaneo(&st, &m, 3000, GAIN, VMAX, &s);
  gc_procesarSimultaneo(&st, &m, 3600, GAIN, VMAX, &s);
  CHECK(s.objetivo[0] == 225 && s.objetivo[1] == 135 &&
        s.objetivo[2] == 135 && s.objetivo[3] == 225,
        "preset A espejado: {225,135,135,225}");

  /* OPTIONS -> peticion de nivelado */
  m = mandoNeutro();
  m.options = true;
  gc_procesarSimultaneo(&st, &m, 4000, GAIN, VMAX, &s);
  CHECK(s.nivelar, "OPTIONS pide nivelado");

  /* Robustez ante desbordamiento de millis */
  gc_EstadoSimultaneo st2;
  gc_simultaneoReset(&st2);
  gc_EstadoMando m2 = mandoNeutro();
  m2.b = true;
  gc_procesarSimultaneo(&st2, &m2, 0xFFFFFF00u, GAIN, VMAX, &s);
  gc_procesarSimultaneo(&st2, &m2, 0x00000100u, GAIN, VMAX, &s); /* +768 ms */
  CHECK(s.usarPosicion, "preset activa a traves del wrap de millis");
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
  testSimultaneoBasico();
  testSimultaneoEmergenciaYLenta();
  testSimultaneoPresets();

  printf("\n=== Resultado: %d/%d pruebas OK", pruebas - fallos, pruebas);
  if (fallos > 0) {
    printf(" — %d FALLOS ===\n", fallos);
    return 1;
  }
  printf(" ===\n");
  return 0;
}
