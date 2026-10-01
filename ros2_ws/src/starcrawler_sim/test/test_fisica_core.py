"""La fisica de MuJoCo: reposo, avance, giro, brazos y el escalon de 20 cm."""

import math

import pytest

mujoco = pytest.importorskip('mujoco')

from starcrawler_sim import fisica_core, mundo_core  # noqa: E402
from terreno_prueba import GEO_PRUEBA, V, VEL_BRAZO  # noqa: E402

LLANO = 'formato: 1\nnombre: llano\nelementos: []\n'
ESCALON = ('formato: 1\nnombre: escalon\nelementos:\n'
           '  - {tipo: caja, pos: [1.0, 0], largo: 1.0, alto: 0.20, ancho: 1.2}\n')
DT = 0.02


def correr(f, segundos, v_izq, v_der, q, objetivo=None):
    """q (lista, se actualiza) va hacia objetivo a la velocidad del firmware."""
    e = None
    for _ in range(int(round(segundos / DT))):
        if objetivo:
            for i in range(4):
                q[i] += max(-VEL_BRAZO * DT, min(VEL_BRAZO * DT, objetivo[i] - q[i]))
        e = f.paso(DT, v_izq, v_der, q)
    return e


@pytest.fixture
def llano():
    return fisica_core.Fisica(GEO_PRUEBA, mundo_core.cargar(LLANO))


def test_reposo_en_llano(llano):
    e = correr(llano, 1.0, 0.0, 0.0, [0.0] * 4)
    assert e.pose.z == pytest.approx(GEO_PRUEBA.orugas.altura_reposo, abs=0.005)
    assert abs(e.pose.cabeceo) < 0.01 and abs(e.pose.balanceo) < 0.01
    assert all(e.apoya) and not e.panza and e.margen > 0.2
    assert not (e.cayendo or e.volcado or e.bloqueado)


def test_avanza_lo_que_mandan_las_orugas(llano):
    correr(llano, 0.5, 0.0, 0.0, [0.0] * 4)
    x0 = llano.estado().pose.x
    e = correr(llano, 4.0, V, V, [0.0] * 4)
    assert e.pose.x - x0 == pytest.approx(4.0 * V, rel=0.15)
    assert abs(e.pose.yaw) < 0.02


def test_gira_a_la_izquierda_con_la_derecha_mas_rapida(llano):
    e = correr(llano, 4.0, -V, V, [0.0] * 4)
    assert e.pose.yaw > 0.1


def test_los_brazos_siguen_la_consigna(llano):
    q = [0.4, 0.4, 0.0, 0.0]
    e = correr(llano, 2.0, 0.0, 0.0, q)
    assert e.elevaciones[0] == pytest.approx(0.4, abs=0.03)
    assert e.elevaciones[1] == pytest.approx(0.4, abs=0.03)


def test_brazos_bajados_levantan_el_chasis(llano):
    e = correr(llano, 5.0, 0.0, 0.0, [0.0] * 4, objetivo=[-0.3] * 4)
    assert e.pose.z > GEO_PRUEBA.orugas.altura_reposo + 0.05


def test_el_escalon_no_se_sube_con_los_brazos_planos():
    mundo = mundo_core.cargar(ESCALON)
    f = fisica_core.Fisica(GEO_PRUEBA, mundo)
    e = correr(f, 25.0, V, V, [0.0] * 4)
    assert mundo.altura(e.pose.x, e.pose.y) == 0.0     # el chasis sigue abajo


def test_el_escalon_se_sube_levantando_y_bajando_los_brazos():
    mundo = mundo_core.cargar(ESCALON)
    f = fisica_core.Fisica(GEO_PRUEBA, mundo)
    d = math.radians
    q = [0.0] * 4
    correr(f, 8.0, 0.0, 0.0, q, [d(35), d(35), 0.0, 0.0])      # morro arriba
    correr(f, 32.0, V, V, q, [d(35), d(35), 0.0, 0.0])         # al canto
    correr(f, 15.0, V, V, q, [d(-15), d(-15), 0.0, 0.0])       # empuja arriba
    e = correr(f, 15.0, V, V, q, [d(-15), d(-15), d(-25), d(-25)])
    assert e.pose.x > 1.0 and e.pose.z > 0.2 + 0.03
    assert not e.volcado
