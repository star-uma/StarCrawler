/*
 * idf_compat.h - StarCrawler - diferencias entre versiones de ESP-IDF
 * ====================================================================
 * micro_ros_setup (rama humble) construye con ESP-IDF v4.1. El resto del
 * codigo usa los nombres de la 4.4 y aqui se traducen. Ver README.md.
 */
#pragma once

#include "esp_idf_version.h"
#include "driver/i2c.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
#include "driver/twai.h"
#include "esp_rom_sys.h"
#else
/* CAN: mismo driver, nombre antiguo */
#include "driver/can.h"
#define twai_general_config_t         can_general_config_t
#define twai_timing_config_t          can_timing_config_t
#define twai_filter_config_t          can_filter_config_t
#define twai_message_t                can_message_t
#define TWAI_GENERAL_CONFIG_DEFAULT   CAN_GENERAL_CONFIG_DEFAULT
#define TWAI_TIMING_CONFIG_1MBITS     CAN_TIMING_CONFIG_1MBITS
#define TWAI_FILTER_CONFIG_ACCEPT_ALL CAN_FILTER_CONFIG_ACCEPT_ALL
#define TWAI_MODE_NORMAL              CAN_MODE_NORMAL
#define twai_driver_install           can_driver_install
#define twai_start                    can_start
#define twai_transmit                 can_transmit
#define twai_receive                  can_receive

#include "esp32/rom/ets_sys.h"
#define esp_rom_delay_us              ets_delay_us
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
/* i2c_master_*_device llegaron en la 4.4; aqui con la API de comandos */
static inline esp_err_t i2c_master_write_to_device(
        i2c_port_t port, uint8_t addr, const uint8_t *data, size_t len,
        TickType_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, timeout);
    i2c_cmd_link_delete(cmd);
    return r;
}

static inline esp_err_t i2c_master_write_read_device(
        i2c_port_t port, uint8_t addr, const uint8_t *wr, size_t wlen,
        uint8_t *rd, size_t rlen, TickType_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write(cmd, (uint8_t *)wr, wlen, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_READ), true);
    i2c_master_read(cmd, rd, rlen, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, timeout);
    i2c_cmd_link_delete(cmd);
    return r;
}
#endif
