"""Tests del reposo de contacto_core (§5.5): z, cabeceo y balanceo con la
planta fija, con colocar(). Sin ROS."""
import math
import random

import pytest

from starcrawler_odometry.chasis_core import pose_chasis
from starcrawler_sim import contacto_core as cc
from terreno_prueba import BETA, GEO_PRUEBA, L, R1, R2, Terreno, bajada, llano

g = math.radians


def colocar(grados, terreno=None, x=0.0, y=0.0, yaw=0.0):
    return cc.colocar(x, y, yaw, [g(a) for a in grados], terreno or llano(), GEO_PRUEBA)


def maxima_penetracion(e):
    """De las restricciones de apoyo: ninguna puede quedar por encima de z."""
    return max([0.0] + [-c.holgura for c in e.contactos if c.tipo == 'apoyo'])


# --- Llano ---------------------------------------------------------------

def test_en_llano_reposa_sobre_las_cuatro_poleas():
    e = colocar((0, 0, 0, 0))
    assert e.pose.z == pytest.approx(R1, abs=1e-9)
    assert e.pose.cabeceo == pytest.approx(0.0, abs=1e-9)
    assert e.pose.balanceo == pytest.approx(0.0, abs=1e-9)
    assert e.apoya == (True, True, True, True)
    assert not e.sin_traccion and not e.panza and not e.volcado
    assert e.margen > 0.2


@pytest.mark.parametrize('q, z', [(-45, R2 + L * math.sin(g(45))), (-90, R2 + L)])
def test_con_todo_bajado_se_pone_de_pie(q, z):
    e = colocar((q, q, q, q))
    assert e.pose.z == pytest.approx(z, abs=1e-6)
    assert round(z, 5) in (0.29606, 0.4015)
    assert e.pose.cabeceo == pytest.approx(0.0, abs=1e-9)


PARIDAD = [((0, 0, 0, 0), 0.0), ((45, 45, 45, 45), 0.0), ((-45, -45, -45, -45), 0.0),
           ((-90, -90, -90, -90), 0.0), ((-30, -30, 0, 0), -8.056),
           ((0, 0, -30, -30), 8.056), ((45, 45, -45, -45), 14.345),
           ((-30, -30, 10, 10), -9.016), ((-20, -20, -60, -60), 9.575)]


@pytest.mark.parametrize('grados, cabeceo', PARIDAD)
def test_en_llano_da_lo_mismo_que_chasis_core(grados, cabeceo):
    e = colocar(grados)
    viejo = pose_chasis([g(a) for a in grados], GEO_PRUEBA.orugas)
    assert e.pose.z == pytest.approx(viejo.altura, abs=5e-5)
    assert math.degrees(e.pose.cabeceo) == pytest.approx(math.degrees(viejo.cabeceo), abs=0.01)
    assert math.degrees(e.pose.balanceo) == pytest.approx(math.degrees(viejo.balanceo), abs=0.01)
    assert math.degrees(e.pose.cabeceo) == pytest.approx(cabeceo, abs=0.001)
    assert all(e.apoya)


def test_una_sola_oruga_bajada_apoya_en_tres():
    """Cambio documentado frente a chasis_core, que la dejaba en una diagonal:
    la punta de FR esta en x = 0,61 y el robot se apoya en FR, RR y RL."""
    e = colocar((-30, 0, 0, 0))
    assert e.apoya == (True, False, True, True)
    assert e.margen > 0.0
    assert pose_chasis([g(-30), 0, 0, 0], GEO_PRUEBA.orugas).holguras[2] > 1e-3


# --- Terreno -------------------------------------------------------------

def test_en_una_rampa_de_15_grados_cabecea_15():
    m = math.tan(g(15))
    e = colocar((-math.degrees(BETA),) * 4, Terreno().rampa_x(-3.0, 3.0, 0.0, 6.0 * m))
    assert math.degrees(e.pose.cabeceo) == pytest.approx(-15.0, abs=0.05)   # morro arriba
    assert e.pose.balanceo == pytest.approx(0.0, abs=1e-9)
    assert all(e.apoya) and not e.sin_traccion


def test_una_caja_bajo_las_orugas_izquierdas_balancea():
    e = colocar((0, 0, 0, 0), Terreno().caja(-2.0, 2.0, 0.05, y0=0.2, y1=2.0))
    assert math.degrees(e.pose.balanceo) == pytest.approx(5.95, abs=0.3)    # izquierda arriba
    assert e.pose.cabeceo == pytest.approx(0.0, abs=1e-6)
    assert e.margen > 0.0


def test_una_barra_bajo_la_panza_la_deja_sin_traccion():
    barra = Terreno().caja(-0.05, 0.05, 0.12, y0=-0.6, y1=0.6)
    e = colocar((30, 30, 30, 30), barra)
    assert e.pose.z == pytest.approx(0.17, abs=1e-9)
    assert e.panza and e.sin_traccion
    assert not any(e.apoya)


def test_con_los_brazos_bajados_se_pone_de_pie_sobre_la_barra():
    barra = Terreno().caja(-0.05, 0.05, 0.12, y0=-0.6, y1=0.6)
    e = colocar((-30, -30, -30, -30), barra)
    assert e.pose.z == pytest.approx(R2 + L * math.sin(g(30)), abs=1e-6)
    assert round(e.pose.z, 4) == 0.2215
    assert all(e.apoya) and not e.panza and not e.sin_traccion


def test_con_el_cdg_detras_del_canto_no_queda_sobre_la_panza():
    """Regresion 1 de §5.8: el prototipo lo dejaba 'en equilibrio' sobre la
    panza, con el CdG 1,6 cm por detras del canto y sin traccion."""
    e = colocar((0, 0, 0, 0), bajada(0.20), x=1.0 - 0.016)
    assert not e.sin_traccion
    assert e.apoya[2] and e.apoya[3]                 # las traseras, arriba
    assert e.panza and e.margen > 0.0
    assert e.pose.cabeceo > 0.0                      # morro abajo, sobre el canto


# --- Propiedades -----------------------------------------------------------

def _terreno_al_azar(rnd):
    t = Terreno()
    for _ in range(rnd.randint(1, 3)):
        x0, y0 = rnd.uniform(-1.0, 1.0), rnd.uniform(-1.0, 1.0)
        x1, y1 = x0 + rnd.uniform(0.05, 1.5), y0 + rnd.uniform(0.05, 1.5)
        if rnd.random() < 0.5:
            t.caja(x0, x1, rnd.uniform(0.01, 0.3), y0=y0, y1=y1)
        else:
            t.rampa_x(x0, x1, rnd.uniform(0.0, 0.3), rnd.uniform(0.0, 0.3), y0=y0, y1=y1)
    return t


def test_propiedad_nunca_penetra_y_el_cdg_queda_dentro():
    rnd = random.Random(11)
    for caso in range(600):
        t = _terreno_al_azar(rnd)
        q = [rnd.uniform(-math.pi / 2, math.pi / 2) for _ in range(4)]
        yaw = rnd.uniform(-math.pi, math.pi)
        e = cc.colocar(0.0, 0.0, yaw, q, t, GEO_PRUEBA)
        assert maxima_penetracion(e) <= 1e-6, caso
        if not e.volcado:
            assert e.margen >= -1e-3, caso
        # colocar no teletransporta: como mucho el empuje de 5 mm
        assert math.hypot(e.pose.x, e.pose.y) <= 0.005 + 1e-6, caso
        assert e.pose.yaw == yaw


@pytest.mark.parametrize('grados, terreno', [
    ((0, 0, 0, 0), llano()),
    ((-30, 0, 20, 0), llano()),
    ((0, 0, 0, 0), Terreno().caja(0.4, 3.0, 0.15)),
    ((10, 10, -20, -20), Terreno().rampa_x(-3.0, 3.0, 0.0, 0.8)),
])
def test_colocar_no_mueve_la_planta(grados, terreno):
    e = colocar(grados, terreno, x=0.3, y=-0.2, yaw=0.7)
    assert (e.pose.x, e.pose.y, e.pose.yaw) == (0.3, -0.2, 0.7)
    assert maxima_penetracion(e) <= 1e-9
