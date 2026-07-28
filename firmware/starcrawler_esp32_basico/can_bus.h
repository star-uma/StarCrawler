/*
 * can_bus.h
 * =========
 * Capa de abstracción del bus CAN a 1 Mbps para los motores RMD-X8.
 * Dos backends seleccionables en config.h:
 *   - MCP2515 externo por SPI (librería ACAN2515, modo polling)
 *   - TWAI interno del ESP32 + transceptor 3.3 V
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Inicializa el bus a 1 Mbps. Devuelve false si el controlador no responde. */
bool canbus_init();

/* Envía una trama estándar de 8 bytes. No bloqueante (cola interna). */
bool canbus_enviar(uint32_t id, const uint8_t datos[8]);

/* Lee una trama pendiente si la hay. Devuelve false si no hay nada. */
bool canbus_recibir(uint32_t *id, uint8_t datos[8]);

/* Mantenimiento del backend (polling del MCP2515). Llamar cada ciclo. */
void canbus_atender();
