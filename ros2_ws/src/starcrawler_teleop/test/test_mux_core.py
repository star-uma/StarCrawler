"""
Tests del multiplexor de /crawler/command.

No necesitan ROS:  pytest ros2_ws/src/starcrawler_teleop/test
Las fuentes son las de config/mux.yaml.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from starcrawler_teleop.mux_core import (      # noqa: E402
    EMERGENCIA, Fuente, Mux, Orden, leer_fuentes)

FUENTES = (
    Fuente('joy', 10, 0.5),
    Fuente('web', 20, 0.25, engancha=True),
    Fuente('joy_activo', 30, 0.25),
)

AVANZA = Orden(incremento=(1, 1, 0, 0))
BAJA = Orden(incremento=(-1, -1, 0, 0))
PARA = Orden(emergencia=True)
REPOSO = Orden()


def mux():
    return Mux(FUENTES)


# ─── Prioridad y frescura ────────────────────────────────────────────────────

def test_sale_la_fuente_de_mas_prioridad():
    m = mux()
    assert m.recibir('joy', REPOSO, 0.0) == REPOSO
    assert m.recibir('web', AVANZA, 0.01) == AVANZA
    assert m.recibir('joy', REPOSO, 0.02) is None
    assert m.recibir('joy_activo', BAJA, 0.03) == BAJA
    assert m.recibir('web', AVANZA, 0.04) is None


def test_la_frescura_es_por_hora_de_llegada():
    m = mux()
    m.recibir('web', AVANZA, 0.0)
    assert m.recibir('joy', REPOSO, 0.25) is None      # web aun fresca
    assert m.recibir('joy', REPOSO, 0.26) == REPOSO    # web caducada


def test_una_fuente_que_vuelve_recupera_el_mando():
    m = mux()
    m.recibir('web', AVANZA, 0.0)
    m.recibir('joy', REPOSO, 1.0)
    assert m.recibir('web', AVANZA, 1.01) == AVANZA


def test_reloj_hacia_atras_caduca():
    m = mux()
    m.recibir('web', AVANZA, 10.0)
    assert m.recibir('joy', REPOSO, 5.0) == REPOSO


def test_fuente_desconocida():
    with pytest.raises(KeyError):
        mux().recibir('otra', REPOSO, 0.0)


# ─── Emergencias ─────────────────────────────────────────────────────────────

def test_la_emergencia_de_una_fuente_inferior_sale_al_momento():
    m = mux()
    m.recibir('web', AVANZA, 0.0)
    assert m.recibir('joy', PARA, 0.01) == EMERGENCIA


def test_or_de_las_emergencias_frescas():
    m = mux()
    m.recibir('joy', PARA, 0.0)
    assert m.recibir('web', AVANZA, 0.01) == EMERGENCIA
    assert m.recibir('web', AVANZA, 0.4) == EMERGENCIA
    # SHARE suelto: el teleop vuelve a mandar sin emergencia
    m.recibir('joy', REPOSO, 0.45)
    assert m.recibir('web', AVANZA, 0.46) == AVANZA


def test_una_emergencia_caducada_no_cuenta():
    m = mux()
    m.recibir('joy', PARA, 0.0)
    assert m.recibir('web', AVANZA, 0.51) == AVANZA


def test_la_web_engancha():
    m = mux()
    assert m.recibir('web', PARA, 0.0) == EMERGENCIA
    assert m.enganchado
    assert m.recibir('web', AVANZA, 0.01) is None
    assert m.recibir('joy_activo', AVANZA, 0.02) is None
    assert m.tick(0.03) == EMERGENCIA


def test_joy_no_engancha():
    m = mux()
    assert m.recibir('joy', PARA, 0.0) == EMERGENCIA
    assert m.recibir('joy_activo', PARA, 0.0) == EMERGENCIA
    assert not m.enganchado
    assert m.tick(0.01) is None


def test_con_el_enganche_puesto_recibir_no_publica():
    m = mux()
    m.recibir('web', PARA, 0.0)
    assert m.recibir('web', PARA, 0.01) is None
    assert m.recibir('joy', PARA, 0.02) is None


def test_tick_solo_con_el_enganche_puesto():
    m = mux()
    m.recibir('web', AVANZA, 0.0)
    assert m.tick(0.01) is None
    m.recibir('web', PARA, 0.02)
    assert m.tick(0.05) == EMERGENCIA
    assert m.tick(0.10) == EMERGENCIA


def test_que_la_web_caduque_no_rearma():
    m = mux()
    m.recibir('web', PARA, 0.0)
    assert m.tick(60.0) == EMERGENCIA
    assert m.activa(60.0) == 'emergencia'
    assert m.recibir('joy', REPOSO, 60.0) is None


def test_emergencia_sale_limpia():
    m = mux()
    sucia = Orden(incremento=(1, -1, 1, -1), usar_posicion=True,
                  objetivo_rad=(1.0, 1.0, 1.0, 1.0), emergencia=True)
    salida = m.recibir('joy', sucia, 0.0)
    assert salida == Orden((0, 0, 0, 0), False, (0.0, 0.0, 0.0, 0.0), True)
    m.recibir('web', sucia, 0.01)
    assert m.tick(0.02) == salida


# ─── Rearme ──────────────────────────────────────────────────────────────────

def test_rearmar_quita_el_enganche():
    m = mux()
    m.recibir('web', PARA, 0.0)
    assert m.rearmar(1.0)
    assert not m.enganchado
    assert m.tick(1.01) is None
    assert m.recibir('joy', REPOSO, 1.02) == REPOSO


def test_el_rearme_se_rechaza_con_una_emergencia_fresca():
    m = mux()
    m.recibir('web', PARA, 0.0)
    m.recibir('joy', PARA, 1.0)          # SHARE pulsado
    assert not m.rearmar(1.1)
    assert m.enganchado
    assert m.piden_emergencia(1.1) == ['joy']
    m.recibir('joy', REPOSO, 1.2)
    assert m.rearmar(1.3)


def test_el_rearme_se_rechaza_si_la_web_aun_pide_emergencia():
    m = mux()
    m.recibir('web', PARA, 0.0)
    assert not m.rearmar(0.1)
    assert m.rearmar(0.3)


def test_rearmar_sin_enganche():
    m = mux()
    assert m.rearmar(0.0)
    m.recibir('joy', PARA, 0.0)
    assert not m.rearmar(0.1)


# ─── activa ──────────────────────────────────────────────────────────────────

def test_valores_de_activa():
    m = mux()
    assert m.activa(0.0) == ''
    m.recibir('joy', REPOSO, 0.0)
    assert m.activa(0.0) == 'joy'
    m.recibir('web', AVANZA, 0.1)
    assert m.activa(0.1) == 'web'
    m.recibir('joy_activo', AVANZA, 0.2)
    assert m.activa(0.2) == 'joy_activo'
    assert m.activa(0.46) == 'joy'       # caducan joy_activo y web
    assert m.activa(0.51) == ''
    m.recibir('web', PARA, 1.0)
    assert m.activa(1.0) == 'emergencia'
    assert m.activa(10.0) == 'emergencia'


def test_activa_con_share_pulsado_es_la_fuente():
    m = mux()
    m.recibir('joy_activo', PARA, 0.0)
    assert m.activa(0.0) == 'joy_activo'


# ─── Tabla de fuentes ────────────────────────────────────────────────────────

def test_fuentes_no_validas():
    with pytest.raises(ValueError):
        Mux([])
    with pytest.raises(ValueError):
        Mux([Fuente('a', 10, 0.5), Fuente('b', 10, 0.5)])
    with pytest.raises(ValueError):
        Mux([Fuente('a', 10, 0.5), Fuente('a', 20, 0.5)])
    with pytest.raises(ValueError):
        Mux([Fuente('emergencia', 10, 0.5)])
    with pytest.raises(ValueError):
        Mux([Fuente('a', 10, 0.0)])


def test_leer_fuentes():
    fuentes = leer_fuentes({
        'joy.topic': 'crawler/command_joy', 'joy.timeout': 0.5,
        'joy.priority': 10,
        'web.topic': 'crawler/command_web', 'web.timeout': 1,
        'web.priority': 20, 'web.engancha': True,
    })
    assert fuentes == [
        (Fuente('joy', 10, 0.5, False), 'crawler/command_joy'),
        (Fuente('web', 20, 1.0, True), 'crawler/command_web'),
    ]


@pytest.mark.parametrize('parametros', [
    {'web.topic': 'x', 'web.timeout': 0.5},                        # falta
    {'web.topic': 'x', 'web.timeout': 0.5, 'web.priority': 1,
     'web.engancho': True},                                        # errata
    {'web.topic': 'x', 'web.timeout': '0.5', 'web.priority': 1},
    {'web.topic': 'x', 'web.timeout': True, 'web.priority': 1},
    {'web.topic': 'x', 'web.timeout': 0.5, 'web.priority': 1.0},
    {'web.topic': '', 'web.timeout': 0.5, 'web.priority': 1},
    {'web.topic': 'x', 'web.timeout': 0.5, 'web.priority': 1,
     'web.engancha': 'true'},
    {'a.web.topic': 'x'},
    {'topic': 'x'},
])
def test_leer_fuentes_rechaza(parametros):
    with pytest.raises(ValueError):
        leer_fuentes(parametros)
