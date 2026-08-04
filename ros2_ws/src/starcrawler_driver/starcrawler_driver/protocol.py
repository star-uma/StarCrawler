"""
protocol.py — protocolo serie PC <-> ESP32 de StarCrawler
=========================================================
Modulo PURO: no importa rclpy ni pyserial, asi que se puede probar con
pytest sin ROS ni hardware (test/test_protocol.py).

Es el espejo exacto de firmware/starcrawler_esp32_ros2/proto.h; los vectores
de prueba de ambos lados coinciden byte a byte (test_proto.cpp en C hace la
misma trama).

Trama
-----
    A5 5A | tipo | len | payload[len] | crc16_lo crc16_hi

    crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) sobre
            tipo + len + payload.
    tipo 0x01 = CMD   (PC -> ESP32), len 20
    tipo 0x02 = STATE (ESP32 -> PC), len 20

Convenciones de unidades
------------------------
* En el bus serie los angulos van en **grados de encoder x100** (cdeg), donde
  180 deg = oruga horizontal, igual que en todo el firmware y el TFG.
* En ROS los angulos van en **rad de elevacion**: positivo = brazo levantado,
  0 = horizontal. Las funciones enc_a_elevacion / elevacion_a_enc hacen la
  traduccion (incluido el espejado de FL y RR).
* Velocidades de traccion en el bus: dps x100 (cdps) del eje de salida.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional

# ─── Constantes de trama ──────────────────────────────────────────────────────

SOF0 = 0xA5
SOF1 = 0x5A

TIPO_CMD = 0x01
TIPO_STATE = 0x02

LEN_CMD = 20
LEN_STATE = 20
TAM_TRAMA_CMD = 4 + LEN_CMD + 2      # 26
TAM_TRAMA_STATE = 4 + LEN_STATE + 2  # 26

N_ORUGAS = 4

# Flags de CMD
FLAG_CMD_USAR_POSICION = 1 << 0
FLAG_CMD_EMERGENCIA = 1 << 1

# Flags de STATE
FLAG_ST_SEGURIDAD = 1 << 0
FLAG_ST_IMU_OK = 1 << 1

# Bits de error (control_core.h)
ERR_ENCODER = 0x0F
ERR_IMU = 1 << 4
ERR_CAN = 1 << 5
ERR_WATCHDOG = 1 << 6

# Orden canonico de las orugas y signo del espejado.
# FR y RL: elevacion = 180 - angulo.  FL y RR: elevacion = angulo - 180.
ORUGAS = ("FR", "FL", "RR", "RL")
SIGNO_ESPEJO = (+1, -1, -1, +1)


# ─── CRC-16/CCITT-FALSE ───────────────────────────────────────────────────────

def crc16(datos: bytes) -> int:
    crc = 0xFFFF
    for b in datos:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# ─── Conversiones de angulo ───────────────────────────────────────────────────

def enc_a_elevacion(angulo_enc_deg: float, idx: int) -> float:
    """Grados de encoder -> rad de elevacion (positivo = brazo levantado)."""
    return math.radians(SIGNO_ESPEJO[idx] * (180.0 - angulo_enc_deg))


def elevacion_a_enc(elevacion_rad: float, idx: int) -> float:
    """Rad de elevacion -> grados de encoder."""
    return 180.0 - SIGNO_ESPEJO[idx] * math.degrees(elevacion_rad)


def incremento_a_bruto(incremento: int, idx: int) -> int:
    """Incremento en elevacion (+1 sube) -> incremento en angulo de encoder.

    Subir FR baja su angulo de encoder; subir FL lo aumenta. Coincide con la
    TABLA_SUBIR del firmware standalone.
    """
    signo = 1 if incremento > 0 else (-1 if incremento < 0 else 0)
    return -SIGNO_ESPEJO[idx] * signo


# ─── Estructuras ──────────────────────────────────────────────────────────────

@dataclass
class Comando:
    """Consigna PC -> ESP32, en unidades de bus (cdps / cdeg de encoder)."""
    vel_izq_cdps: int = 0
    vel_der_cdps: int = 0
    incremento: List[int] = field(default_factory=lambda: [0] * N_ORUGAS)
    objetivo_cdeg: List[int] = field(default_factory=lambda: [18000] * N_ORUGAS)
    usar_posicion: bool = False
    emergencia: bool = False
    seq: int = 0


@dataclass
class Estado:
    """Telemetria ESP32 -> PC."""
    angulo_cdeg: List[int] = field(default_factory=lambda: [0] * N_ORUGAS)
    vel_izq_cdps: int = 0
    vel_der_cdps: int = 0
    error_bits: int = 0
    seguridad: bool = False
    imu_ok: bool = False
    seq_eco: int = 0
    roll_cdeg: int = 0
    pitch_cdeg: int = 0

    # --- Vistas en unidades ROS ---
    def angulos_rad(self) -> List[float]:
        return [enc_a_elevacion(self.angulo_cdeg[i] / 100.0, i)
                for i in range(N_ORUGAS)]

    def encoders_ok(self) -> List[bool]:
        return [not (self.error_bits >> i) & 1 for i in range(N_ORUGAS)]

    @property
    def can_ok(self) -> bool:
        return not self.error_bits & ERR_CAN


# ─── Serializacion ────────────────────────────────────────────────────────────

def _clamp_i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def empaquetar_comando(cmd: Comando) -> bytes:
    """Comando -> trama completa lista para escribir en el puerto serie."""
    flags = 0
    if cmd.usar_posicion:
        flags |= FLAG_CMD_USAR_POSICION
    if cmd.emergencia:
        flags |= FLAG_CMD_EMERGENCIA

    payload = struct.pack(
        "<hh4b4hBBH",
        _clamp_i16(cmd.vel_izq_cdps),
        _clamp_i16(cmd.vel_der_cdps),
        *[max(-1, min(1, int(v))) for v in cmd.incremento],
        *[_clamp_i16(v) for v in cmd.objetivo_cdeg],
        flags & 0xFF,
        cmd.seq & 0xFF,
        0,  # reservado
    )
    assert len(payload) == LEN_CMD, len(payload)

    cabecera = struct.pack("<BB", TIPO_CMD, LEN_CMD)
    return bytes([SOF0, SOF1]) + cabecera + payload + \
        struct.pack("<H", crc16(cabecera + payload))


def desempaquetar_comando(payload: bytes) -> Comando:
    """Payload de una trama CMD ya validada -> Comando (lo usa el simulador
    y el firmware hace lo mismo en C)."""
    if len(payload) != LEN_CMD:
        raise ValueError(f"payload CMD de {len(payload)} bytes, esperados {LEN_CMD}")
    campos = struct.unpack("<hh4b4hBBH", payload)
    flags = campos[10]
    return Comando(
        vel_izq_cdps=campos[0],
        vel_der_cdps=campos[1],
        incremento=list(campos[2:6]),
        objetivo_cdeg=list(campos[6:10]),
        usar_posicion=bool(flags & FLAG_CMD_USAR_POSICION),
        emergencia=bool(flags & FLAG_CMD_EMERGENCIA),
        seq=campos[11],
    )


def desempaquetar_estado(payload: bytes) -> Estado:
    """Payload de una trama STATE ya validada -> Estado."""
    if len(payload) != LEN_STATE:
        raise ValueError(f"payload STATE de {len(payload)} bytes, esperados {LEN_STATE}")
    campos = struct.unpack("<4hhhHBBhh", payload)
    return Estado(
        angulo_cdeg=list(campos[0:4]),
        vel_izq_cdps=campos[4],
        vel_der_cdps=campos[5],
        error_bits=campos[6],
        seguridad=bool(campos[7] & FLAG_ST_SEGURIDAD),
        imu_ok=bool(campos[7] & FLAG_ST_IMU_OK),
        seq_eco=campos[8],
        roll_cdeg=campos[9],
        pitch_cdeg=campos[10],
    )


def empaquetar_estado(st: Estado) -> bytes:
    """Solo para tests y simulador: Estado -> trama STATE completa."""
    flags = 0
    if st.seguridad:
        flags |= FLAG_ST_SEGURIDAD
    if st.imu_ok:
        flags |= FLAG_ST_IMU_OK

    payload = struct.pack(
        "<4hhhHBBhh",
        *[_clamp_i16(v) for v in st.angulo_cdeg],
        _clamp_i16(st.vel_izq_cdps),
        _clamp_i16(st.vel_der_cdps),
        st.error_bits & 0xFFFF,
        flags & 0xFF,
        st.seq_eco & 0xFF,
        _clamp_i16(st.roll_cdeg),
        _clamp_i16(st.pitch_cdeg),
    )
    cabecera = struct.pack("<BB", TIPO_STATE, LEN_STATE)
    return bytes([SOF0, SOF1]) + cabecera + payload + \
        struct.pack("<H", crc16(cabecera + payload))


# ─── Parser incremental (el serie es un flujo, no datagramas) ────────────────

class Parser:
    """Maquina de estados que extrae tramas de un flujo de bytes.

    Tolera basura, arranques a media trama y bytes de sincronismo dentro del
    payload. Lleva contadores para diagnostico del enlace.

    Usar las fabricas ParserEstado() / ParserComando() en lugar de esta clase.
    """

    ESPERA_SOF0, ESPERA_SOF1, TIPO, LEN, PAYLOAD, CRC_LO, CRC_HI = range(7)

    def __init__(self, tipo_esperado: int, longitud: int, decodificador) -> None:
        self._tipo_esperado = tipo_esperado
        self._longitud = longitud
        self._decodificador = decodificador
        self.tramas_ok = 0
        self.errores_crc = 0
        self.bytes_descartados = 0
        self._reset()

    def _reset(self) -> None:
        self._estado = self.ESPERA_SOF0
        self._tipo = 0
        self._len = 0
        self._payload = bytearray()
        self._crc_rx = 0

    def alimentar(self, datos: bytes) -> list:
        """Procesa un bloque de bytes y devuelve las tramas completas."""
        salida = []
        for b in datos:
            msg = self._byte(b)
            if msg is not None:
                salida.append(msg)
        return salida

    def _byte(self, b: int):
        if self._estado == self.ESPERA_SOF0:
            if b == SOF0:
                self._estado = self.ESPERA_SOF1
            else:
                self.bytes_descartados += 1
        elif self._estado == self.ESPERA_SOF1:
            if b == SOF1:
                self._estado = self.TIPO
            elif b == SOF0:
                self.bytes_descartados += 1  # sigue esperando el segundo SOF
            else:
                self.bytes_descartados += 2
                self._estado = self.ESPERA_SOF0
        elif self._estado == self.TIPO:
            self._tipo = b
            self._estado = self.LEN
        elif self._estado == self.LEN:
            self._len = b
            self._payload = bytearray()
            # Solo aceptamos la trama que nos toca; si no cuadra, resincronizamos
            if self._tipo == self._tipo_esperado and b == self._longitud:
                self._estado = self.PAYLOAD
            else:
                self._reset()
        elif self._estado == self.PAYLOAD:
            self._payload.append(b)
            if len(self._payload) == self._len:
                self._estado = self.CRC_LO
        elif self._estado == self.CRC_LO:
            self._crc_rx = b
            self._estado = self.CRC_HI
        elif self._estado == self.CRC_HI:
            crc_rx = self._crc_rx | (b << 8)
            cabecera = bytes([self._tipo, self._len])
            payload = bytes(self._payload)
            calculado = crc16(cabecera + payload)
            self._reset()
            if calculado == crc_rx:
                self.tramas_ok += 1
                return self._decodificador(payload)
            self.errores_crc += 1
        return None


def ParserEstado() -> Parser:
    """Parser de tramas STATE (lo usa el driver del PC)."""
    return Parser(TIPO_STATE, LEN_STATE, desempaquetar_estado)


def ParserComando() -> Parser:
    """Parser de tramas CMD (lo usa el simulador; el firmware hace lo mismo)."""
    return Parser(TIPO_CMD, LEN_CMD, desempaquetar_comando)
