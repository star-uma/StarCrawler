"""
Tests de la logica del mando (esquema simultaneo).

No necesitan ROS ni mando:  pytest ros2_ws/src/starcrawler_teleop/test
Los casos son los mismos que los del firmware en C (test_gamepad_core.cpp),
para que ambas implementaciones no se separen.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from starcrawler_teleop.joy_logic import (      # noqa: E402
    Ajustes, LogicaMando, Mapeo, deadzone)

N_EJES = 8
N_BOTONES = 14


def ejes(**kwargs):
    v = [0.0] * N_EJES
    for k, val in kwargs.items():
        v[int(k[1:])] = val      # a1=..., a6=...
    return v


def botones(*pulsados):
    v = [0] * N_BOTONES
    for i in pulsados:
        v[i] = 1
    return v


def logica():
    return LogicaMando(Mapeo(), Ajustes())


# ─── Deadzone ────────────────────────────────────────────────────────────────

def test_deadzone_dentro_es_cero():
    assert deadzone(0.05, 0.08) == 0.0
    assert deadzone(-0.079, 0.08) == 0.0


def test_deadzone_fondo_de_escala():
    assert deadzone(1.0, 0.08) == pytest.approx(1.0)
    assert deadzone(-1.0, 0.08) == pytest.approx(-1.0)


def test_deadzone_reescala_desde_el_borde():
    assert deadzone(0.54, 0.08) == pytest.approx(0.5)


# ─── Traccion ────────────────────────────────────────────────────────────────

def test_reposo_no_manda_nada():
    s = logica().procesar(ejes(), botones(), 0.0)
    assert s.lineal == 0.0 and s.angular == 0.0
    assert s.incremento == [0, 0, 0, 0]
    assert not s.usar_posicion and not s.emergencia


def test_avance_adelante():
    a = Ajustes()
    s = logica().procesar(ejes(a1=-1.0), botones(), 0.0)   # stick arriba = -1
    assert s.lineal == pytest.approx(a.max_lineal)
    assert s.angular == pytest.approx(0.0)


def test_marcha_atras():
    a = Ajustes()
    s = logica().procesar(ejes(a1=1.0), botones(), 0.0)
    assert s.lineal == pytest.approx(-a.max_lineal)


def test_giro_a_la_derecha_es_angular_negativo():
    """Convencion ROS: angular.z positivo = giro a la izquierda."""
    a = Ajustes()
    s = logica().procesar(ejes(a2=1.0), botones(), 0.0)
    assert s.angular == pytest.approx(-a.max_angular)
    s = logica().procesar(ejes(a2=-1.0), botones(), 0.0)
    assert s.angular == pytest.approx(a.max_angular)


# ─── Pares de orugas y simultaneidad ─────────────────────────────────────────

def test_traccion_y_orugas_a_la_vez():
    """La razon de ser del esquema: conducir mientras bascula una oruga."""
    a = Ajustes()
    lg = logica()
    s = lg.procesar(ejes(a1=-1.0), botones(4), 0.0)   # avance + L1
    assert s.lineal == pytest.approx(a.max_lineal)
    assert s.incremento == [1, 1, 0, 0]


def test_par_delantero_sube_y_baja():
    lg = logica()
    assert lg.procesar(ejes(), botones(4), 0.0).incremento == [1, 1, 0, 0]
    assert lg.procesar(ejes(), botones(6), 0.1).incremento == [-1, -1, 0, 0]


def test_par_trasero_sube_y_baja():
    lg = logica()
    assert lg.procesar(ejes(), botones(5), 0.0).incremento == [0, 0, 1, 1]
    assert lg.procesar(ejes(), botones(7), 0.1).incremento == [0, 0, -1, -1]


def test_pares_opuestos_se_anulan():
    lg = logica()
    assert lg.procesar(ejes(), botones(4, 6), 0.0).incremento == [0, 0, 0, 0]


def test_los_dos_pares_a_la_vez():
    lg = logica()
    assert lg.procesar(ejes(), botones(4, 5), 0.0).incremento == [1, 1, 1, 1]


def test_gatillo_como_eje_analogico():
    m = Mapeo()
    m.eje_l2 = 3
    lg = LogicaMando(m, Ajustes())
    assert lg.procesar(ejes(a3=0.8), botones(), 0.0).incremento == [-1, -1, 0, 0]
    assert lg.procesar(ejes(a3=-1.0), botones(), 0.1).incremento == [0, 0, 0, 0]


# ─── Cruceta ─────────────────────────────────────────────────────────────────

def test_cruceta_inclina_el_conjunto():
    lg = logica()
    # arriba: suben las delanteras -> inclinar adelante
    assert lg.procesar(ejes(a7=1.0), botones(), 0.0).incremento == [1, 1, -1, -1]
    assert lg.procesar(ejes(a7=-1.0), botones(), 0.1).incremento == [-1, -1, 1, 1]
    assert lg.procesar(ejes(a6=-1.0), botones(), 0.2).incremento == [-1, 1, -1, 1]
    assert lg.procesar(ejes(a6=1.0), botones(), 0.3).incremento == [1, -1, 1, -1]


def test_cruceta_en_diagonal_no_hace_nada():
    lg = logica()
    s = lg.procesar(ejes(a6=1.0, a7=1.0), botones(), 0.0)
    assert s.incremento == [0, 0, 0, 0]


def test_cruceta_como_botones():
    m = Mapeo()
    m.dpad_botones = (12, 13, 10, 11)
    m.boton_l3 = -1          # 11 pasa a ser cruceta en este mapeo
    lg = LogicaMando(m, Ajustes())
    assert lg.procesar(ejes(), botones(12), 0.0).incremento == [1, 1, -1, -1]


def test_cruceta_sentinela_menos_uno_equivale_a_ejes():
    """El nodo ROS declara dpad_botones=[-1,-1,-1,-1] (rclpy no tipa listas
    vacias): debe comportarse exactamente igual que la cruceta por ejes."""
    m = Mapeo()
    m.dpad_botones = (-1, -1, -1, -1)
    lg = LogicaMando(m, Ajustes())
    assert lg.procesar(ejes(a7=1.0), botones(), 0.0).incremento == [1, 1, -1, -1]
    assert lg.procesar(ejes(), botones(12), 0.1).incremento == [0, 0, 0, 0]


def test_cruceta_con_eje_y_invertido():
    m = Mapeo()
    m.dpad_y_arriba_positivo = False
    lg = LogicaMando(m, Ajustes())
    assert lg.procesar(ejes(a7=-1.0), botones(), 0.0).incremento == [1, 1, -1, -1]


# ─── Emergencia, velocidad lenta, deadman ────────────────────────────────────

def test_share_es_parada_de_emergencia():
    lg = logica()
    s = lg.procesar(ejes(a1=-1.0), botones(8, 4), 0.0)
    assert s.emergencia
    assert s.lineal == 0.0 and s.incremento == [0, 0, 0, 0]


def test_l3_alterna_velocidad_lenta():
    a = Ajustes()
    lg = logica()
    s = lg.procesar(ejes(a1=-1.0), botones(11), 0.0)
    assert s.velocidad_lenta
    assert s.lineal == pytest.approx(a.max_lineal * a.factor_lento)
    # mantenerlo pulsado no vuelve a conmutar
    s = lg.procesar(ejes(a1=-1.0), botones(11), 0.1)
    assert s.velocidad_lenta
    lg.procesar(ejes(), botones(), 0.2)          # soltar
    s = lg.procesar(ejes(a1=-1.0), botones(11), 0.3)
    assert not s.velocidad_lenta
    assert s.lineal == pytest.approx(a.max_lineal)


def test_deadman_bloquea_si_no_se_pulsa():
    m = Mapeo()
    m.boton_enable = 10
    lg = LogicaMando(m, Ajustes())
    s = lg.procesar(ejes(a1=-1.0), botones(), 0.0)
    assert s.lineal == 0.0
    s = lg.procesar(ejes(a1=-1.0), botones(10), 0.1)
    assert s.lineal != 0.0


def test_options_pide_nivelado():
    assert logica().procesar(ejes(), botones(9), 0.0).nivelar


# ─── Presets de pose ─────────────────────────────────────────────────────────

def test_preset_necesita_mantener_el_boton():
    lg = logica()
    assert not lg.procesar(ejes(), botones(1), 0.0).usar_posicion
    assert not lg.procesar(ejes(), botones(1), 0.4).usar_posicion
    assert lg.procesar(ejes(), botones(1), 0.55).usar_posicion


def test_presets_dan_la_misma_elevacion_en_las_cuatro():
    """En elevacion la pose es simetrica: el espejado lo hace el driver."""
    esperado = {0: -45.0, 1: 0.0, 3: 45.0, 2: 90.0}
    for boton, grados in esperado.items():
        lg = logica()
        lg.procesar(ejes(), botones(boton), 0.0)
        s = lg.procesar(ejes(), botones(boton), 0.6)
        assert s.usar_posicion
        for v in s.objetivo_rad:
            assert math.degrees(v) == pytest.approx(grados)


def test_preset_persiste_al_soltar():
    lg = logica()
    lg.procesar(ejes(), botones(1), 0.0)
    lg.procesar(ejes(), botones(1), 0.6)
    s = lg.procesar(ejes(), botones(), 0.7)
    assert s.usar_posicion


def test_entrada_manual_cancela_el_preset():
    lg = logica()
    lg.procesar(ejes(), botones(1), 0.0)
    assert lg.procesar(ejes(), botones(1), 0.6).usar_posicion
    s = lg.procesar(ejes(), botones(4), 0.7)          # L1
    assert not s.usar_posicion
    assert s.incremento == [1, 1, 0, 0]
    # y sigue cancelado despues
    assert not lg.procesar(ejes(), botones(), 0.8).usar_posicion


def test_preset_no_manda_incrementos():
    lg = logica()
    lg.procesar(ejes(), botones(1), 0.0)
    s = lg.procesar(ejes(), botones(1), 0.6)
    assert s.incremento == [0, 0, 0, 0]


def test_cambiar_de_preset_reinicia_el_temporizador():
    lg = logica()
    lg.procesar(ejes(), botones(1), 0.0)
    s = lg.procesar(ejes(), botones(2), 0.4)     # cambia de boton
    assert not s.usar_posicion
    assert lg.procesar(ejes(), botones(2), 1.0).usar_posicion
