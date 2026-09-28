"""
Tests de config/mux.yaml: la tabla de fuentes de twist_mux y la de
crawler_mux tienen que ser la misma. No necesitan ROS.
"""
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from starcrawler_teleop.mux_core import Mux, leer_fuentes   # noqa: E402

RUTA = os.path.join(os.path.dirname(__file__), '..', 'config', 'mux.yaml')

WATCHDOG_ESP32_S = 0.3      # WATCHDOG_TIMEOUT_MS de la app de micro-ROS


def seccion(nodo):
    with open(RUTA, encoding='utf-8') as f:
        return yaml.safe_load(f)[nodo]['ros__parameters']


def aplanar(topics):
    """{'web': {'topic': ...}} -> {'web.topic': ...}, como los ve el nodo."""
    return {f'{n}.{campo}': v
            for n, tabla in topics.items() for campo, v in tabla.items()}


@pytest.fixture
def twist():
    return seccion('twist_mux')['topics']


@pytest.fixture
def crawler():
    return {f.nombre: (f, topico) for f, topico in
            leer_fuentes(aplanar(seccion('crawler_mux')['topics']))}


def test_crawler_mux_acepta_su_seccion(crawler):
    Mux(f for f, _ in crawler.values())


def test_twist_mux_sin_stamped():
    assert seccion('twist_mux')['use_stamped'] is False


def test_mismas_fuentes_en_las_dos_secciones(twist, crawler):
    # navigation no tiene orugas
    assert set(twist) - {'navigation'} == set(crawler)


def test_mismas_prioridades_y_timeouts(twist, crawler):
    for nombre, (fuente, _) in crawler.items():
        assert twist[nombre]['priority'] == fuente.prioridad, nombre
        assert float(twist[nombre]['timeout']) == fuente.timeout_s, nombre


def test_topicos_emparejados(twist, crawler):
    # cmd_vel_X va con crawler/command_X
    for nombre, (_, topico) in crawler.items():
        vel = twist[nombre]['topic']
        assert vel.startswith('cmd_vel_'), nombre
        assert topico == 'crawler/command' + vel[len('cmd_vel'):], nombre


def test_web_y_joy_activo_caducan_antes_que_el_watchdog(crawler):
    assert crawler['web'][0].timeout_s < WATCHDOG_ESP32_S
    assert crawler['joy_activo'][0].timeout_s < WATCHDOG_ESP32_S


def test_el_ds4_tocado_gana_a_la_web(crawler):
    assert (crawler['joy_activo'][0].prioridad > crawler['web'][0].prioridad
            > crawler['joy'][0].prioridad)


def test_solo_engancha_la_web(crawler):
    assert [n for n, (f, _) in crawler.items() if f.engancha] == ['web']
