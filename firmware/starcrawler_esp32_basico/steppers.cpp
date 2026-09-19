/*
 * steppers.cpp
 * ============
 * Generación de pasos por timer hardware + ISR. Ver steppers.h.
 *
 * El timer corre a un tick fijo (TICK_ISR_US) y cada motor cuenta ticks
 * hasta su propio semiperiodo, así que los cuatro pueden ir a velocidades
 * distintas con un solo temporizador. Sobre eso se monta la rampa de
 * aceleración: cada motor arranca a SEMIPERIODO_ARRANQUE_US y acelera
 * hasta SEMIPERIODO_STEP_US.
 */
#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "config.h"
#include "steppers.h"
#include "control_core.h"

static const int pinStep[CC_NUM_ORUGAS] = PINES_STEP;
static const int pinDir[CC_NUM_ORUGAS]  = PINES_DIR;
static const int pinEna[CC_NUM_ORUGAS]  = PINES_ENA;

static const int dirHorario[CC_NUM_ORUGAS]     = TABLA_DIR_HORARIO;
static const int dirAntihorario[CC_NUM_ORUGAS] = TABLA_DIR_ANTIHORARIO;

/* Semiperiodos expresados en ticks del temporizador */
static const uint16_t TICKS_REGIMEN =
    (uint16_t)(SEMIPERIODO_STEP_US / TICK_ISR_US);
#if RAMPA_ACTIVA
static const uint16_t TICKS_ARRANQUE =
    (uint16_t)(SEMIPERIODO_ARRANQUE_US / TICK_ISR_US);
#else
static const uint16_t TICKS_ARRANQUE = TICKS_REGIMEN;
#endif

/* Máscaras precalculadas por banco de GPIO (0..31 y 32..39) */
static uint32_t mascaraBanco0[CC_NUM_ORUGAS];
static uint32_t mascaraBanco1[CC_NUM_ORUGAS];

static volatile bool motorActivo[CC_NUM_ORUGAS] = {false, false, false, false};
static volatile bool nivelStep[CC_NUM_ORUGAS]   = {false, false, false, false};
static volatile uint16_t periodoTicks[CC_NUM_ORUGAS];
static volatile uint16_t contadorTicks[CC_NUM_ORUGAS];

/* Sentido que tiene ahora cada motor: 0 parado, +1 y -1 en marcha.
 * Solo se toca el pin DIR cuando cambia (ver steppers_comando). */
static int8_t sentidoActual[CC_NUM_ORUGAS] = {0, 0, 0, 0};

static hw_timer_t *timerSteppers = NULL;

/* ISR: cada motor cuenta ticks hasta su semiperiodo y alterna su pin STEP.
 * Solo toca registros GPIO — sin llamadas a la API de Arduino. */
static void IRAM_ATTR steppers_isr() {
  uint32_t set0 = 0, clr0 = 0, set1 = 0, clr1 = 0;

  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (!motorActivo[i]) continue;
    if (++contadorTicks[i] < periodoTicks[i]) continue;
    contadorTicks[i] = 0;

    nivelStep[i] = !nivelStep[i];
    if (nivelStep[i]) {
      set0 |= mascaraBanco0[i];
      set1 |= mascaraBanco1[i];
    } else {
      clr0 |= mascaraBanco0[i];
      clr1 |= mascaraBanco1[i];
      /* Pulso completo terminado: acelerar acortando el semiperiodo */
      if (periodoTicks[i] > TICKS_REGIMEN) {
        const uint16_t margen = periodoTicks[i] - TICKS_REGIMEN;
        periodoTicks[i] -= (margen < RAMPA_DECREMENTO_TICKS)
                               ? margen
                               : RAMPA_DECREMENTO_TICKS;
      }
    }
  }

  if (set0) GPIO.out_w1ts = set0;
  if (clr0) GPIO.out_w1tc = clr0;
  if (set1) GPIO.out1_w1ts.val = set1;
  if (clr1) GPIO.out1_w1tc.val = clr1;
}

void steppers_init() {
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    pinMode(pinStep[i], OUTPUT);
    pinMode(pinDir[i], OUTPUT);
    pinMode(pinEna[i], OUTPUT);

    digitalWrite(pinStep[i], LOW);
    digitalWrite(pinDir[i], LOW);
    /* Arranque en estado seguro: drivers deshabilitados */
    digitalWrite(pinEna[i], HIGH);

    periodoTicks[i] = TICKS_ARRANQUE;
    contadorTicks[i] = 0;

    if (pinStep[i] < 32) {
      mascaraBanco0[i] = 1UL << pinStep[i];
      mascaraBanco1[i] = 0;
    } else {
      mascaraBanco0[i] = 0;
      mascaraBanco1[i] = 1UL << (pinStep[i] - 32);
    }
  }

  /* Timer a 1 MHz, interrupción cada TICK_ISR_US.
   * Compatibilidad con las dos APIs de timer del core ESP32. */
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  timerSteppers = timerBegin(1000000);
  timerAttachInterrupt(timerSteppers, &steppers_isr);
  timerAlarm(timerSteppers, TICK_ISR_US, true, 0);
#else
  timerSteppers = timerBegin(0, 80, true); /* 80 MHz / 80 = 1 MHz */
  timerAttachInterrupt(timerSteppers, &steppers_isr, true);
  timerAlarmWrite(timerSteppers, TICK_ISR_US, true);
  timerAlarmEnable(timerSteppers);
#endif
}

void steppers_comando(int motor, int8_t cmd) {
  if (motor < 0 || motor >= CC_NUM_ORUGAS) return;

  if (cmd == 0) {
    motorActivo[motor] = false;
    sentidoActual[motor] = 0;
#if PARADA_LIBERA_DRIVER
    digitalWrite(pinEna[motor], HIGH); /* driver deshabilitado (original) */
#else
    digitalWrite(pinEna[motor], LOW);  /* mantiene par de retención */
#endif
    return;
  }

  const int8_t sentido = (cmd > 0) ? 1 : -1;

  /* El pin DIR solo se toca al arrancar o al invertir. Escribirlo en cada
   * ciclo de control, de forma asíncrona a los pulsos, puede cambiarlo justo
   * en un flanco de STEP y hacer que el driver dé un paso hacia el lado
   * contrario: el DM542 pide DIR estable antes del pulso. */
  if (sentidoActual[motor] != sentido) {
    motorActivo[motor] = false;   /* que la ISR no pulse mientras cambia DIR */
    digitalWrite(pinDir[motor],
                 (sentido > 0) ? dirHorario[motor] : dirAntihorario[motor]);
    delayMicroseconds(10);        /* margen DIR->STEP del DM542 */
    periodoTicks[motor] = TICKS_ARRANQUE;  /* rearmar la rampa */
    contadorTicks[motor] = 0;
    sentidoActual[motor] = sentido;
  }

  digitalWrite(pinEna[motor], LOW); /* driver habilitado */
  motorActivo[motor] = true;
}

void steppers_pararTodos() {
  for (int i = 0; i < CC_NUM_ORUGAS; i++) steppers_comando(i, 0);
}

bool steppers_algunoActivo() {
  for (int i = 0; i < CC_NUM_ORUGAS; i++) {
    if (motorActivo[i]) return true;
  }
  return false;
}
