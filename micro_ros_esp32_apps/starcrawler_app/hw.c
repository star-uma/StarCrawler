/*
 * hw.c - StarCrawler - capa de hardware sobre ESP-IDF
 * ====================================================================
 * Port de can_bus.cpp, steppers.cpp y encoders.cpp de Arduino a ESP-IDF.
 * Las firmas no cambian (ver hw.h), asi que control_core.c se reutiliza
 * sin tocarlo.
 *
 * VERSION DE ESP-IDF: escrito con los nombres de la 4.4; las diferencias
 * con la 4.1 (la que trae micro_ros_setup humble) van en idf_compat.h.
 */

#include "hw.h"
#include "config.h"
#include "control_core.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "idf_compat.h"

/* Con HW_SIMULADO (config.h) no se toca ni un pin: el CAN no se instala, los
 * pulsos de la ISR mueven brazos simulados y los encoders leen esos brazos. */
#if HW_SIMULADO
#define PIN(p, v) ((void)(p), (void)(v))
#else
#define PIN(p, v) gpio_set_level((gpio_num_t)(p), (v))
#endif

/* ==================================================================== */
/*  Utilidades                                                          */
/* ==================================================================== */

uint32_t hw_millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ==================================================================== */
/*  CAN (TWAI)                                                          */
/* ==================================================================== */

#if HW_SIMULADO

/* RMD simulados: siguen la consigna con un retardo de primer orden y
 * contestan cada trama como los de verdad (id + 0x100, mismo comando, la
 * velocidad en dps enteros). La cola es la del TWAI: si se llena, se pierde. */
#define SIM_RMD_TAU_S   0.05f
#define SIM_RMD_COLA    16

static const uint32_t idsRmd[CC_NUM_ORUGAS] = {
    CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL
};
static float    consignaRmd[CC_NUM_ORUGAS];
static float    velRmd[CC_NUM_ORUGAS];
static bool     libreRmd[CC_NUM_ORUGAS] = {true, true, true, true};
static uint32_t colaId[SIM_RMD_COLA];
static uint8_t  colaDatos[SIM_RMD_COLA][8];
static int      colaIni, colaN;
static int64_t  usAnterior;
/* liberarTraccion() tambien se llama desde el nucleo 0 antes de reiniciar */
static portMUX_TYPE muxCan = portMUX_INITIALIZER_UNLOCKED;

bool canbus_init(void) {
    usAnterior = esp_timer_get_time();
    return true;
}

bool canbus_enviar(uint32_t id, const uint8_t datos[8]) {
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        if (id != idsRmd[i]) continue;
        uint8_t r[8] = {datos[0], 0, 0, 0, 0, 0, 0, 0};
        portENTER_CRITICAL(&muxCan);
        if (datos[0] == 0xA2) {
            const int32_t c = (int32_t)((uint32_t)datos[4] |
                                        ((uint32_t)datos[5] << 8) |
                                        ((uint32_t)datos[6] << 16) |
                                        ((uint32_t)datos[7] << 24));
            consignaRmd[i] = (float)c / 100.0f;
            libreRmd[i] = false;
            const int16_t v = (int16_t)lrintf(velRmd[i]);
            r[1] = 30;                                  /* temperatura, C */
            r[4] = (uint8_t)((uint16_t)v & 0xFF);
            r[5] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
        } else if (datos[0] == 0x80) {
            libreRmd[i] = true;
        }
        if (colaN < SIM_RMD_COLA) {
            const int k = (colaIni + colaN) % SIM_RMD_COLA;
            colaId[k] = id + 0x100;
            memcpy(colaDatos[k], r, 8);
            colaN++;
        }
        portEXIT_CRITICAL(&muxCan);
        return true;
    }
    return true;
}

bool canbus_recibir(uint32_t *id, uint8_t datos[8]) {
    bool hay = false;
    portENTER_CRITICAL(&muxCan);
    if (colaN > 0) {
        *id = colaId[colaIni];
        memcpy(datos, colaDatos[colaIni], 8);
        colaIni = (colaIni + 1) % SIM_RMD_COLA;
        colaN--;
        hay = true;
    }
    portEXIT_CRITICAL(&muxCan);
    return hay;
}

/* Integra la velocidad de los motores; el liberado se para por rozamiento */
void canbus_atender(void) {
    const int64_t us = esp_timer_get_time();
    float dt = (float)(us - usAnterior) * 1e-6f;
    usAnterior = us;
    if (dt > 0.1f) dt = 0.1f;
    const float k = dt / (SIM_RMD_TAU_S + dt);
    portENTER_CRITICAL(&muxCan);
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        const float objetivo = libreRmd[i] ? 0.0f : consignaRmd[i];
        velRmd[i] += (objetivo - velRmd[i]) * k;
    }
    portEXIT_CRITICAL(&muxCan);
}

#else

bool canbus_init(void) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX, (gpio_num_t)PIN_TWAI_RX, TWAI_MODE_NORMAL);
    /* En seguridad se liberan los cuatro en cada ciclo: 4 respuestas / 10 ms */
    g.tx_queue_len = 8;
    g.rx_queue_len = 16;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

/* Con el bus caido no se encola nada: el driver de IDF casca (assert en
 * twai.c) si quedan tramas pendientes al recuperarse. Como en a09f202. */
bool canbus_enviar(uint32_t id, const uint8_t datos[8]) {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK || st.state != TWAI_STATE_RUNNING) {
        return false;
    }
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = id;
    msg.data_length_code = 8;
    for (int i = 0; i < 8; i++) msg.data[i] = datos[i];
    return twai_transmit(&msg, 0) == ESP_OK;
}

bool canbus_recibir(uint32_t *id, uint8_t datos[8]) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;
    *id = msg.identifier;
    for (int i = 0; i < 8; i++) datos[i] = msg.data[i];
    return true;
}

/* Bus-off: vaciar colas, recuperar y volver a arrancar cuando el
 * controlador quede parado. */
void canbus_atender(void) {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return;
    if (st.state == TWAI_STATE_BUS_OFF) {
        twai_clear_transmit_queue();
        twai_clear_receive_queue();
        twai_initiate_recovery();
    } else if (st.state == TWAI_STATE_STOPPED) {
        twai_start();
    }
}

#endif /* HW_SIMULADO */

/* ==================================================================== */
/*  Steppers                                                            */
/* ==================================================================== */

/* Misma estructura que steppers.cpp: un unico timer a tick fijo y cada
 * motor contando ticks hasta su propio semiperiodo, con rampa de
 * aceleracion. Lo unico que cambia es de donde sale la interrupcion y
 * como se escriben los pines. */

static const int pinStep[CC_NUM_ORUGAS] = PINES_STEP;
static const int pinDir[CC_NUM_ORUGAS]  = PINES_DIR;
static const int pinEna[CC_NUM_ORUGAS]  = PINES_ENA;

static const int dirHorario[CC_NUM_ORUGAS]     = TABLA_DIR_HORARIO;
static const int dirAntihorario[CC_NUM_ORUGAS] = TABLA_DIR_ANTIHORARIO;

static const uint16_t TICKS_REGIMEN =
    (uint16_t)(SEMIPERIODO_STEP_US / TICK_ISR_US);
#if RAMPA_ACTIVA
static const uint16_t TICKS_ARRANQUE =
    (uint16_t)(SEMIPERIODO_ARRANQUE_US / TICK_ISR_US);
#else
static const uint16_t TICKS_ARRANQUE = TICKS_REGIMEN;
#endif

static volatile bool     motorActivo[CC_NUM_ORUGAS];
static volatile bool     nivelStep[CC_NUM_ORUGAS];
static volatile uint16_t periodoTicks[CC_NUM_ORUGAS];
static volatile uint16_t contadorTicks[CC_NUM_ORUGAS];
static volatile int8_t   sentidoActual[CC_NUM_ORUGAS];

#if HW_SIMULADO
/* Pulsos dados por cada brazo: la posicion del brazo simulado */
static volatile int32_t  pulsosSim[CC_NUM_ORUGAS];
#endif

#define TIMER_GRUPO  TIMER_GROUP_0
#define TIMER_IDX    TIMER_0

static bool IRAM_ATTR steppers_isr(void *arg) {
    (void)arg;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        if (!motorActivo[i]) continue;
        if (++contadorTicks[i] < periodoTicks[i]) continue;
        contadorTicks[i] = 0;

        nivelStep[i] = !nivelStep[i];
        PIN(pinStep[i], nivelStep[i] ? 1 : 0);
#if HW_SIMULADO
        if (nivelStep[i]) pulsosSim[i] += sentidoActual[i];
#endif

        if (!nivelStep[i] && periodoTicks[i] > TICKS_REGIMEN) {
            /* Pulso completo terminado: acelerar acortando el semiperiodo */
            const uint16_t margen = periodoTicks[i] - TICKS_REGIMEN;
            periodoTicks[i] -= (margen < RAMPA_DECREMENTO_TICKS)
                                   ? margen : RAMPA_DECREMENTO_TICKS;
        }
    }
    return false;   /* sin cambio de contexto */
}

void steppers_init(void) {
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
#if !HW_SIMULADO
        gpio_reset_pin((gpio_num_t)pinStep[i]);
        gpio_reset_pin((gpio_num_t)pinDir[i]);
        gpio_reset_pin((gpio_num_t)pinEna[i]);
        gpio_set_direction((gpio_num_t)pinStep[i], GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)pinDir[i],  GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)pinEna[i],  GPIO_MODE_OUTPUT);
#endif

        PIN(pinStep[i], 0);
        PIN(pinDir[i], 0);
        /* Arranque en estado seguro: drivers deshabilitados */
        PIN(pinEna[i], 1);

        motorActivo[i]   = false;
        nivelStep[i]     = false;
        periodoTicks[i]  = TICKS_ARRANQUE;
        contadorTicks[i] = 0;
        sentidoActual[i] = 0;
    }

    timer_config_t cfg = {
        .divider     = 80,            /* 80 MHz / 80 = 1 MHz -> 1 us */
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GRUPO, TIMER_IDX, &cfg);
    timer_set_counter_value(TIMER_GRUPO, TIMER_IDX, 0);
    timer_set_alarm_value(TIMER_GRUPO, TIMER_IDX, TICK_ISR_US);
    timer_enable_intr(TIMER_GRUPO, TIMER_IDX);
    timer_isr_callback_add(TIMER_GRUPO, TIMER_IDX, steppers_isr, NULL,
                           ESP_INTR_FLAG_IRAM);
    timer_start(TIMER_GRUPO, TIMER_IDX);
}

void steppers_comando(int motor, int8_t cmd) {
    if (motor < 0 || motor >= CC_NUM_ORUGAS) return;

    if (cmd == 0) {
        motorActivo[motor]   = false;
        sentidoActual[motor] = 0;
        /* STEP en bajo: si queda en alto, la primera conmutacion del
         * siguiente arranque es un flanco de bajada y no da paso. */
        nivelStep[motor] = false;
        PIN(pinStep[motor], 0);
#if PARADA_LIBERA_DRIVER
        PIN(pinEna[motor], 1);
#else
        PIN(pinEna[motor], 0);
#endif
        return;
    }

    const int8_t sentido = (cmd > 0) ? 1 : -1;

    /* DIR solo se toca al arrancar o al invertir: escribirlo de forma
     * asincrona a los pulsos puede caer en un flanco de STEP y hacer que
     * el DM542 de un paso hacia el lado contrario. */
    if (sentidoActual[motor] != sentido) {
        motorActivo[motor] = false;
        PIN(pinDir[motor], (sentido > 0) ? dirHorario[motor]
                                         : dirAntihorario[motor]);
        esp_rom_delay_us(10);              /* margen DIR->STEP del DM542 */
        periodoTicks[motor]  = TICKS_ARRANQUE;
        contadorTicks[motor] = 0;
        sentidoActual[motor] = sentido;
    }

    PIN(pinEna[motor], 0);   /* driver habilitado */
    motorActivo[motor] = true;
}

void steppers_pararTodos(void) {
    for (int i = 0; i < CC_NUM_ORUGAS; i++) steppers_comando(i, 0);
}

bool steppers_algunoActivo(void) {
    for (int i = 0; i < CC_NUM_ORUGAS; i++) if (motorActivo[i]) return true;
    return false;
}

/* ==================================================================== */
/*  Encoders (AS5600 tras multiplexor TCA9548A)                         */
/* ==================================================================== */

#define I2C_PUERTO      I2C_NUM_0
/* Cota si el bus I2C se cuelga. En ticks y nunca menos de 2: a 100 Hz,
 * pdMS_TO_TICKS(5) es 0 y 1 tick puede vencer al instante. */
#define I2C_TIMEOUT_MS     5
#define I2C_TIMEOUT_TICKS  (pdMS_TO_TICKS(I2C_TIMEOUT_MS) >= 2 \
                            ? pdMS_TO_TICKS(I2C_TIMEOUT_MS) : 2)

static const float offsetsEncoder[CC_NUM_ORUGAS] = OFFSETS_ENCODER;

#if HW_SIMULADO

/* 400 pulsos por vuelta en el DM542 y reductora 1:80
 * (docs/cadena_de_elevacion.md). Los brazos arrancan horizontales. */
#define SIM_GRADOS_POR_PULSO   (360.0f / (400.0f * 80.0f))
#define SIM_ANGULO_INICIAL_DEG 180.0f

void encoders_init(void) {}

/* Como el AS5600: 12 bits por vuelta y el mismo offset que el robot */
bool encoders_leer(int idx, float *angDeg) {
    if (idx < 0 || idx >= CC_NUM_ORUGAS) return false;
    const float brazo = SIM_ANGULO_INICIAL_DEG
                      + (float)pulsosSim[idx] * SIM_GRADOS_POR_PULSO;
    long crudo = lrintf((brazo - offsetsEncoder[idx]) / 0.087890625f) % 4096;
    if (crudo < 0) crudo += 4096;
    *angDeg = cc_as5600ADeg((uint16_t)crudo, offsetsEncoder[idx]);
    return true;
}

#else

void encoders_init(void) {
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_PUERTO, &cfg);
    i2c_driver_install(I2C_PUERTO, I2C_MODE_MASTER, 0, 0, 0);
}

/* Abre un canal del multiplexor. Todo lo que se hable despues por I2C va
 * al encoder de ese canal. */
static bool tcaSeleccionar(uint8_t canal) {
    if (canal > 7) return false;
    const uint8_t mascara = (uint8_t)(1u << canal);
    return i2c_master_write_to_device(
               I2C_PUERTO, DIR_TCA9548A, &mascara, 1,
               I2C_TIMEOUT_TICKS) == ESP_OK;
}

bool encoders_leer(int idx, float *angDeg) {
    if (idx < 0 || idx >= CC_NUM_ORUGAS) return false;
    if (!tcaSeleccionar((uint8_t)idx)) return false;

    /* RAW ANGLE en 0x0C (alto) y 0x0D (bajo). Los dos bytes se leen en
     * una unica transaccion para que la muestra sea coherente: leerlos
     * por separado puede mezclar dos muestras distintas. */
    const uint8_t reg = 0x0C;
    uint8_t buf[2];
    if (i2c_master_write_read_device(
            I2C_PUERTO, DIR_AS5600, &reg, 1, buf, 2,
            I2C_TIMEOUT_TICKS) != ESP_OK) {
        return false;
    }

    const uint16_t crudo = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
    *angDeg = cc_as5600ADeg(crudo, offsetsEncoder[idx]);
    return true;
}

#endif /* HW_SIMULADO */
