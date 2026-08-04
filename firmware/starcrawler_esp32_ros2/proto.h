/*
 * proto.h — protocolo serie ESP32 <-> PC (ROS 2)
 * ==============================================
 * Espejo exacto de ros2_ws/src/starcrawler_driver/starcrawler_driver/protocol.py.
 * Las tramas de ambos lados coinciden byte a byte (ver test/host/test_proto.cpp
 * y test_protocol.py: comparten el mismo vector canonico).
 *
 * Trama:  A5 5A | tipo | len | payload[len] | crc16_lo crc16_hi
 *         crc16 = CRC-16/CCITT-FALSE sobre tipo + len + payload
 *
 * Sin dependencias de Arduino: se compila tambien en el PC para los tests.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SP_SOF0 0xA5
#define SP_SOF1 0x5A

#define SP_TIPO_CMD   0x01
#define SP_TIPO_STATE 0x02

#define SP_LEN_CMD   20
#define SP_LEN_STATE 20
#define SP_TAM_TRAMA_CMD   (4 + SP_LEN_CMD + 2)   /* 26 */
#define SP_TAM_TRAMA_STATE (4 + SP_LEN_STATE + 2) /* 26 */

#define SP_N_ORUGAS 4

/* Flags de CMD */
#define SP_FLAG_CMD_USAR_POSICION (1u << 0)
#define SP_FLAG_CMD_EMERGENCIA    (1u << 1)

/* Flags de STATE */
#define SP_FLAG_ST_SEGURIDAD (1u << 0)
#define SP_FLAG_ST_IMU_OK    (1u << 1)

/* Consigna PC -> ESP32. Angulos en grados de ENCODER x100 (180 deg =
 * horizontal) e incrementos en el sentido del encoder: la traduccion desde
 * la convencion de elevacion de ROS la hace el driver del PC. */
typedef struct {
  int16_t vel_izq_cdps;
  int16_t vel_der_cdps;
  int8_t  incremento[SP_N_ORUGAS];
  int16_t objetivo_cdeg[SP_N_ORUGAS];
  uint8_t flags;
  uint8_t seq;
} sp_Comando;

/* Telemetria ESP32 -> PC */
typedef struct {
  int16_t  angulo_cdeg[SP_N_ORUGAS];
  int16_t  vel_izq_cdps;
  int16_t  vel_der_cdps;
  uint16_t error_bits;
  uint8_t  flags;
  uint8_t  seq_eco;
  int16_t  roll_cdeg;   /* reservado (sin IMU en esta variante) */
  int16_t  pitch_cdeg;  /* reservado */
} sp_Estado;

uint16_t sp_crc16(const uint8_t *datos, size_t n);

/* Serializan una trama completa (cabecera + payload + CRC).
 * Devuelven el numero de bytes escritos. */
int sp_empaquetarEstado(const sp_Estado *st, uint8_t out[SP_TAM_TRAMA_STATE]);
int sp_empaquetarComando(const sp_Comando *cmd, uint8_t out[SP_TAM_TRAMA_CMD]);

/* Payload ya validado -> struct */
void sp_desempaquetarComando(const uint8_t payload[SP_LEN_CMD], sp_Comando *out);
void sp_desempaquetarEstado(const uint8_t payload[SP_LEN_STATE], sp_Estado *out);

/* ── Parser incremental ─────────────────────────────────────────────────────
 * El serie es un flujo de bytes: hay que resincronizar solo. Tolera basura,
 * arranques a media trama y bytes de sincronismo dentro del payload.
 */
typedef struct {
  uint8_t  estado;
  uint8_t  tipo_esperado;
  uint8_t  len_esperado;
  uint8_t  idx;
  uint8_t  payload[SP_LEN_CMD > SP_LEN_STATE ? SP_LEN_CMD : SP_LEN_STATE];
  uint16_t crc_rx;
  /* Contadores de diagnostico */
  uint32_t tramas_ok;
  uint32_t errores_crc;
  uint32_t bytes_descartados;
} sp_Parser;

/* tipo_esperado: SP_TIPO_CMD en el ESP32, SP_TIPO_STATE en el PC. */
void sp_parserInit(sp_Parser *p, uint8_t tipo_esperado, uint8_t longitud);

/* Alimenta un byte. Devuelve true cuando se ha completado una trama valida,
 * y entonces deja el payload crudo en 'payload_out' (SP_LEN_* bytes). */
bool sp_parserByte(sp_Parser *p, uint8_t b, uint8_t *payload_out);

#ifdef __cplusplus
}
#endif
