/*
 * hw.c - StarCrawler - capa de hardware sobre ESP-IDF
 * ====================================================================
 * Port de can_bus.cpp, steppers.cpp y encoders.cpp de Arduino a ESP-IDF.
 * Las firmas no cambian (ver hw.h), asi que control_core.c se reutiliza
 * sin tocarlo.
 *
 * VERSION DE ESP-IDF: escrito contra la v4.4, que es lo que usa el
 * camino freertos/esp32 de micro_ros_setup. Las tres APIs elegidas son
 * las de esa rama:
 *   - driver/twai.h    CAN interno (en IDF < 4.2 se llamaba driver/can.h,
 *                      que es lo que usa Donatello)
 *   - driver/timer.h   timer de grupo (en IDF 5.x pasa a gptimer)
 *   - driver/i2c.h     API legacy (en IDF 5.2+ pasa a i2c_master)
 * Si el entorno resulta ser otra version, estas tres son las que hay
 * que tocar; el resto del fichero no depende de la version.
 *
 * SIN VERIFICAR: no se ha compilado nunca.
 */

#include "hw.h"
#include "config.h"
#include "control_core.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/timer.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/* ==================================================================== */
/*  Utilidades                                                          */
/* ==================================================================== */

uint32_t hw_millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ==================================================================== */
/*  CAN (TWAI)                                                          */
/* ==================================================================== */

bool canbus_init(void) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX, (gpio_num_t)PIN_TWAI_RX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 8;
    g.rx_queue_len = 8;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

bool canbus_enviar(uint32_t id, const uint8_t datos[8]) {
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
static int8_t            sentidoActual[CC_NUM_ORUGAS];

#define TIMER_GRUPO  TIMER_GROUP_0
#define TIMER_IDX    TIMER_0

static bool IRAM_ATTR steppers_isr(void *arg) {
    (void)arg;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        if (!motorActivo[i]) continue;
        if (++contadorTicks[i] < periodoTicks[i]) continue;
        contadorTicks[i] = 0;

        nivelStep[i] = !nivelStep[i];
        gpio_set_level((gpio_num_t)pinStep[i], nivelStep[i] ? 1 : 0);

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
        gpio_reset_pin((gpio_num_t)pinStep[i]);
        gpio_reset_pin((gpio_num_t)pinDir[i]);
        gpio_reset_pin((gpio_num_t)pinEna[i]);
        gpio_set_direction((gpio_num_t)pinStep[i], GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)pinDir[i],  GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)pinEna[i],  GPIO_MODE_OUTPUT);

        gpio_set_level((gpio_num_t)pinStep[i], 0);
        gpio_set_level((gpio_num_t)pinDir[i], 0);
        /* Arranque en estado seguro: drivers deshabilitados */
        gpio_set_level((gpio_num_t)pinEna[i], 1);

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
        gpio_set_level((gpio_num_t)pinStep[motor], 0);
#if PARADA_LIBERA_DRIVER
        gpio_set_level((gpio_num_t)pinEna[motor], 1);
#else
        gpio_set_level((gpio_num_t)pinEna[motor], 0);
#endif
        return;
    }

    const int8_t sentido = (cmd > 0) ? 1 : -1;

    /* DIR solo se toca al arrancar o al invertir: escribirlo de forma
     * asincrona a los pulsos puede caer en un flanco de STEP y hacer que
     * el DM542 de un paso hacia el lado contrario. */
    if (sentidoActual[motor] != sentido) {
        motorActivo[motor] = false;
        gpio_set_level((gpio_num_t)pinDir[motor],
                       (sentido > 0) ? dirHorario[motor]
                                     : dirAntihorario[motor]);
        esp_rom_delay_us(10);              /* margen DIR->STEP del DM542 */
        periodoTicks[motor]  = TICKS_ARRANQUE;
        contadorTicks[motor] = 0;
        sentidoActual[motor] = sentido;
    }

    gpio_set_level((gpio_num_t)pinEna[motor], 0);   /* driver habilitado */
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
#define I2C_TIMEOUT_MS  20

static const float offsetsEncoder[CC_NUM_ORUGAS] = OFFSETS_ENCODER;

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
               pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
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
            pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK) {
        return false;
    }

    const uint16_t crudo = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
    *angDeg = cc_as5600ADeg(crudo, offsetsEncoder[idx]);
    return true;
}
