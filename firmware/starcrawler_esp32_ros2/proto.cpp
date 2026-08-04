/*
 * proto.cpp — implementacion del protocolo serie. Ver proto.h.
 * Serializacion campo a campo (nada de memcpy de structs): asi el formato
 * del bus no depende del empaquetado del compilador.
 */
#include "proto.h"

#include <string.h>

/* ── CRC-16/CCITT-FALSE ─────────────────────────────────────────────────── */

uint16_t sp_crc16(const uint8_t *datos, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)datos[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* ── Helpers little-endian ──────────────────────────────────────────────── */

static void pon_i16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v & 0xFF);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

static int16_t lee_i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void pon_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static uint16_t lee_u16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Cierra la trama: cabecera + payload + CRC */
static int cerrar_trama(uint8_t *out, uint8_t tipo, uint8_t len) {
  out[0] = SP_SOF0;
  out[1] = SP_SOF1;
  out[2] = tipo;
  out[3] = len;
  const uint16_t crc = sp_crc16(out + 2, (size_t)len + 2);
  pon_u16(out + 4 + len, crc);
  return 4 + len + 2;
}

/* ── STATE ──────────────────────────────────────────────────────────────── */

int sp_empaquetarEstado(const sp_Estado *st, uint8_t out[SP_TAM_TRAMA_STATE]) {
  uint8_t *p = out + 4;
  for (int i = 0; i < SP_N_ORUGAS; i++) pon_i16(p + 2 * i, st->angulo_cdeg[i]);
  pon_i16(p + 8, st->vel_izq_cdps);
  pon_i16(p + 10, st->vel_der_cdps);
  pon_u16(p + 12, st->error_bits);
  p[14] = st->flags;
  p[15] = st->seq_eco;
  pon_i16(p + 16, st->roll_cdeg);
  pon_i16(p + 18, st->pitch_cdeg);
  return cerrar_trama(out, SP_TIPO_STATE, SP_LEN_STATE);
}

void sp_desempaquetarEstado(const uint8_t payload[SP_LEN_STATE], sp_Estado *out) {
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    out->angulo_cdeg[i] = lee_i16(payload + 2 * i);
  }
  out->vel_izq_cdps = lee_i16(payload + 8);
  out->vel_der_cdps = lee_i16(payload + 10);
  out->error_bits = lee_u16(payload + 12);
  out->flags = payload[14];
  out->seq_eco = payload[15];
  out->roll_cdeg = lee_i16(payload + 16);
  out->pitch_cdeg = lee_i16(payload + 18);
}

/* ── CMD ────────────────────────────────────────────────────────────────── */

int sp_empaquetarComando(const sp_Comando *cmd, uint8_t out[SP_TAM_TRAMA_CMD]) {
  uint8_t *p = out + 4;
  pon_i16(p + 0, cmd->vel_izq_cdps);
  pon_i16(p + 2, cmd->vel_der_cdps);
  for (int i = 0; i < SP_N_ORUGAS; i++) p[4 + i] = (uint8_t)cmd->incremento[i];
  for (int i = 0; i < SP_N_ORUGAS; i++) pon_i16(p + 8 + 2 * i, cmd->objetivo_cdeg[i]);
  p[16] = cmd->flags;
  p[17] = cmd->seq;
  pon_u16(p + 18, 0); /* reservado */
  return cerrar_trama(out, SP_TIPO_CMD, SP_LEN_CMD);
}

void sp_desempaquetarComando(const uint8_t payload[SP_LEN_CMD], sp_Comando *out) {
  out->vel_izq_cdps = lee_i16(payload + 0);
  out->vel_der_cdps = lee_i16(payload + 2);
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    out->incremento[i] = (int8_t)payload[4 + i];
  }
  for (int i = 0; i < SP_N_ORUGAS; i++) {
    out->objetivo_cdeg[i] = lee_i16(payload + 8 + 2 * i);
  }
  out->flags = payload[16];
  out->seq = payload[17];
}

/* ── Parser ─────────────────────────────────────────────────────────────── */

enum {
  E_SOF0 = 0, E_SOF1, E_TIPO, E_LEN, E_PAYLOAD, E_CRC_LO, E_CRC_HI
};

void sp_parserInit(sp_Parser *p, uint8_t tipo_esperado, uint8_t longitud) {
  memset(p, 0, sizeof(*p));
  p->estado = E_SOF0;
  p->tipo_esperado = tipo_esperado;
  p->len_esperado = longitud;
}

bool sp_parserByte(sp_Parser *p, uint8_t b, uint8_t *payload_out) {
  const uint8_t tipo_esperado = p->tipo_esperado;
  const uint8_t len_esperado = p->len_esperado;

  switch (p->estado) {
    case E_SOF0:
      if (b == SP_SOF0) p->estado = E_SOF1;
      else p->bytes_descartados++;
      break;

    case E_SOF1:
      if (b == SP_SOF1) {
        p->estado = E_TIPO;
      } else if (b == SP_SOF0) {
        p->bytes_descartados++;  /* sigue esperando el segundo SOF */
      } else {
        p->bytes_descartados += 2;
        p->estado = E_SOF0;
      }
      break;

    case E_TIPO:
      /* Solo nos interesa nuestro tipo; si no, resincronizamos */
      if (b == tipo_esperado) {
        p->estado = E_LEN;
      } else {
        p->bytes_descartados += 3;
        p->estado = E_SOF0;
      }
      break;

    case E_LEN:
      if (b == len_esperado) {
        p->idx = 0;
        p->estado = E_PAYLOAD;
      } else {
        p->bytes_descartados += 4;
        p->estado = E_SOF0;
      }
      break;

    case E_PAYLOAD:
      p->payload[p->idx++] = b;
      if (p->idx >= len_esperado) p->estado = E_CRC_LO;
      break;

    case E_CRC_LO:
      p->crc_rx = b;
      p->estado = E_CRC_HI;
      break;

    case E_CRC_HI: {
      p->crc_rx |= (uint16_t)b << 8;
      uint8_t cab[2 + (SP_LEN_CMD > SP_LEN_STATE ? SP_LEN_CMD : SP_LEN_STATE)];
      cab[0] = tipo_esperado;
      cab[1] = len_esperado;
      memcpy(cab + 2, p->payload, len_esperado);
      const uint16_t calculado = sp_crc16(cab, (size_t)len_esperado + 2);
      p->estado = E_SOF0;
      if (calculado == p->crc_rx) {
        p->tramas_ok++;
        if (payload_out) memcpy(payload_out, p->payload, len_esperado);
        return true;
      }
      p->errores_crc++;
      break;
    }

    default:
      p->estado = E_SOF0;
      break;
  }
  return false;
}
