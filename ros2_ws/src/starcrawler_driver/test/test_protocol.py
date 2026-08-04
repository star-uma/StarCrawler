"""
Tests del protocolo serie y del ESP32 simulado.

No necesitan ROS ni hardware:  pytest ros2_ws/src/starcrawler_driver/test
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from starcrawler_driver import protocol as proto            # noqa: E402
from starcrawler_driver.simulator import Esp32Simulado      # noqa: E402


# ─── CRC ─────────────────────────────────────────────────────────────────────

def test_crc16_vector_conocido():
    # Vector estandar de CRC-16/CCITT-FALSE
    assert proto.crc16(b'123456789') == 0x29B1


def test_crc16_vacio():
    assert proto.crc16(b'') == 0xFFFF


# ─── Trama canonica (debe coincidir byte a byte con test_proto.cpp) ─────────

def test_trama_canonica_de_comando():
    """Esta misma trama la genera el firmware en C; si cambia una, falla."""
    cmd = proto.Comando(
        vel_izq_cdps=1234,
        vel_der_cdps=-1234,
        incremento=[1, -1, 0, 1],
        objetivo_cdeg=[22500, 13500, 13500, 22500],
        usar_posicion=True,
        emergencia=False,
        seq=7,
    )
    trama = proto.empaquetar_comando(cmd)
    assert len(trama) == proto.TAM_TRAMA_CMD == 26
    assert trama[:4] == bytes([0xA5, 0x5A, 0x01, 20])
    # Referencia cruzada C <-> Python (ver test/host/test_proto.cpp)
    assert trama.hex() == (
        'a55a0114'          # SOF, tipo, len
        'd204'              # vel_izq = 1234
        '2efb'              # vel_der = -1234
        '01ff0001'          # incrementos 1,-1,0,1
        'e457bc34bc34e457'  # objetivos 22500,13500,13500,22500
        '01'                # flags: usar_posicion
        '07'                # seq
        '0000'              # reservado
        'dd57'              # crc16
    )


def test_ida_y_vuelta_comando():
    cmd = proto.Comando(
        vel_izq_cdps=-4000, vel_der_cdps=4000,
        incremento=[-1, 1, 1, -1],
        objetivo_cdeg=[9000, 27000, 27000, 9000],
        usar_posicion=False, emergencia=True, seq=250)
    trama = proto.empaquetar_comando(cmd)
    vuelta = proto.desempaquetar_comando(trama[4:-2])
    assert vuelta == cmd


def test_ida_y_vuelta_estado():
    st = proto.Estado(
        angulo_cdeg=[18000, 9012, 27099, 13500],
        vel_izq_cdps=-1500, vel_der_cdps=1500,
        error_bits=0x0045, seguridad=True, imu_ok=False, seq_eco=42)
    vuelta = proto.desempaquetar_estado(proto.empaquetar_estado(st)[4:-2])
    assert vuelta == st


def test_saturacion_int16():
    cmd = proto.Comando(vel_izq_cdps=999999, vel_der_cdps=-999999)
    vuelta = proto.desempaquetar_comando(proto.empaquetar_comando(cmd)[4:-2])
    assert vuelta.vel_izq_cdps == 32767
    assert vuelta.vel_der_cdps == -32768


# ─── Conversiones de angulo ──────────────────────────────────────────────────

def test_horizontal_es_cero_en_las_cuatro():
    for i in range(4):
        assert proto.enc_a_elevacion(180.0, i) == pytest.approx(0.0)


def test_pose_vertical_arriba_del_tfg():
    """{FR,FL,RR,RL} = {90,270,270,90} debe dar +90 deg en las cuatro."""
    enc = [90.0, 270.0, 270.0, 90.0]
    for i, a in enumerate(enc):
        assert math.degrees(proto.enc_a_elevacion(a, i)) == pytest.approx(90.0)


def test_pose_vertical_abajo_del_tfg():
    enc = [270.0, 90.0, 90.0, 270.0]
    for i, a in enumerate(enc):
        assert math.degrees(proto.enc_a_elevacion(a, i)) == pytest.approx(-90.0)


def test_conversion_angulo_es_biyectiva():
    for i in range(4):
        for enc in (90.0, 135.0, 180.0, 225.0, 270.0):
            elev = proto.enc_a_elevacion(enc, i)
            assert proto.elevacion_a_enc(elev, i) == pytest.approx(enc)


def test_subir_mapea_al_sentido_correcto_de_encoder():
    """Subir (+1) baja el angulo en FR/RL y lo sube en FL/RR (TABLA_SUBIR)."""
    assert [proto.incremento_a_bruto(1, i) for i in range(4)] == [-1, 1, 1, -1]
    assert [proto.incremento_a_bruto(-1, i) for i in range(4)] == [1, -1, -1, 1]
    assert [proto.incremento_a_bruto(0, i) for i in range(4)] == [0, 0, 0, 0]


# ─── Parser ──────────────────────────────────────────────────────────────────

def test_parser_extrae_tramas_seguidas():
    p = proto.ParserEstado()
    a = proto.Estado(angulo_cdeg=[18000, 18000, 18000, 18000], seq_eco=1)
    b = proto.Estado(angulo_cdeg=[9000, 27000, 27000, 9000], seq_eco=2)
    salida = p.alimentar(proto.empaquetar_estado(a) + proto.empaquetar_estado(b))
    assert [s.seq_eco for s in salida] == [1, 2]
    assert p.tramas_ok == 2 and p.errores_crc == 0


def test_parser_resincroniza_tras_basura():
    p = proto.ParserEstado()
    st = proto.Estado(seq_eco=9)
    salida = p.alimentar(b'\x00basura\xff' + proto.empaquetar_estado(st))
    assert len(salida) == 1 and salida[0].seq_eco == 9
    assert p.bytes_descartados > 0


def test_parser_tolera_trama_partida_en_trozos():
    p = proto.ParserEstado()
    trama = proto.empaquetar_estado(proto.Estado(seq_eco=5))
    for i in range(len(trama) - 1):
        assert p.alimentar(trama[i:i + 1]) == []
    salida = p.alimentar(trama[-1:])
    assert len(salida) == 1 and salida[0].seq_eco == 5


def test_parser_detecta_crc_corrupto():
    p = proto.ParserEstado()
    trama = bytearray(proto.empaquetar_estado(proto.Estado(seq_eco=3)))
    trama[10] ^= 0xFF          # corrompe el payload
    assert p.alimentar(bytes(trama)) == []
    assert p.errores_crc == 1 and p.tramas_ok == 0


def test_parser_ignora_tramas_de_otro_tipo():
    """El driver no debe confundirse con el eco de sus propios comandos."""
    p = proto.ParserEstado()
    assert p.alimentar(proto.empaquetar_comando(proto.Comando())) == []
    st = proto.Estado(seq_eco=4)
    assert len(p.alimentar(proto.empaquetar_estado(st))) == 1


def test_parser_sincroniza_con_sof_suelto_previo():
    """Un 0xA5 huerfano antes de la trama no debe romper el sincronismo."""
    p = proto.ParserEstado()
    salida = p.alimentar(b'\xa5' + proto.empaquetar_estado(proto.Estado(seq_eco=6)))
    assert len(salida) == 1 and salida[0].seq_eco == 6


# ─── ESP32 simulado ──────────────────────────────────────────────────────────

def _bombear(sim, segundos, dt=0.01):
    for _ in range(int(segundos / dt)):
        sim.avanzar(dt)


def test_simulador_arranca_en_seguridad():
    sim = Esp32Simulado()
    _bombear(sim, 0.1)
    st = proto.ParserEstado().alimentar(sim.read(sim.in_waiting))[-1]
    assert st.seguridad
    assert st.error_bits & proto.ERR_WATCHDOG


def test_simulador_sale_de_seguridad_con_comandos():
    sim = Esp32Simulado()
    sim.write(proto.empaquetar_comando(proto.Comando()))
    _bombear(sim, 0.1)
    st = proto.ParserEstado().alimentar(sim.read(sim.in_waiting))[-1]
    assert not st.seguridad


def test_simulador_watchdog_para_al_callar():
    sim = Esp32Simulado()
    sim.write(proto.empaquetar_comando(proto.Comando()))
    _bombear(sim, 0.1)
    sim.read(sim.in_waiting)
    _bombear(sim, 0.5)          # silencio > 300 ms
    st = proto.ParserEstado().alimentar(sim.read(sim.in_waiting))[-1]
    assert st.seguridad


def test_simulador_mueve_orugas_con_incrementos():
    sim = Esp32Simulado()
    p = proto.ParserEstado()
    cmd = proto.Comando(incremento=[1, -1, 0, 0])
    for _ in range(200):        # 2 s a dt=0.01
        sim.write(proto.empaquetar_comando(cmd))
        sim.avanzar(0.01)
    st = p.alimentar(sim.read(sim.in_waiting))[-1]
    # ~4.69 deg/s * 2 s = ~9.4 deg
    assert st.angulo_cdeg[0] / 100.0 == pytest.approx(189.4, abs=0.5)
    assert st.angulo_cdeg[1] / 100.0 == pytest.approx(170.6, abs=0.5)
    assert st.angulo_cdeg[2] / 100.0 == pytest.approx(180.0, abs=0.1)


def test_simulador_respeta_limites_software():
    sim = Esp32Simulado()
    p = proto.ParserEstado()
    cmd = proto.Comando(incremento=[1, 1, 1, 1])
    for _ in range(4000):       # 40 s: de sobra para llegar al tope
        sim.write(proto.empaquetar_comando(cmd))
        sim.avanzar(0.01)
    st = p.alimentar(sim.read(sim.in_waiting))[-1]
    for a in st.angulo_cdeg:
        assert a / 100.0 == pytest.approx(275.0, abs=0.01)


def test_simulador_alcanza_objetivo_de_posicion():
    sim = Esp32Simulado()
    p = proto.ParserEstado()
    cmd = proto.Comando(usar_posicion=True,
                        objetivo_cdeg=[20000, 16000, 18000, 18000])
    for _ in range(1000):       # 10 s
        sim.write(proto.empaquetar_comando(cmd))
        sim.avanzar(0.01)
    st = p.alimentar(sim.read(sim.in_waiting))[-1]
    assert st.angulo_cdeg[0] / 100.0 == pytest.approx(200.0, abs=0.6)
    assert st.angulo_cdeg[1] / 100.0 == pytest.approx(160.0, abs=0.6)


def test_simulador_emergencia_anula_traccion():
    sim = Esp32Simulado()
    p = proto.ParserEstado()
    cmd = proto.Comando(vel_izq_cdps=3000, vel_der_cdps=3000, emergencia=True)
    for _ in range(20):
        sim.write(proto.empaquetar_comando(cmd))
        sim.avanzar(0.01)
    st = p.alimentar(sim.read(sim.in_waiting))[-1]
    assert st.vel_izq_cdps == 0 and st.vel_der_cdps == 0


def test_simulador_sin_encoders_no_mueve_en_posicion():
    """Igual que el firmware: sin realimentacion no hay lazo cerrado."""
    sim = Esp32Simulado(encoders_ok=False)
    p = proto.ParserEstado()
    cmd = proto.Comando(usar_posicion=True, objetivo_cdeg=[9000] * 4)
    for _ in range(300):
        sim.write(proto.empaquetar_comando(cmd))
        sim.avanzar(0.01)
    st = p.alimentar(sim.read(sim.in_waiting))[-1]
    assert st.angulo_cdeg[0] / 100.0 == pytest.approx(180.0, abs=0.1)
    assert st.error_bits & proto.ERR_ENCODER
