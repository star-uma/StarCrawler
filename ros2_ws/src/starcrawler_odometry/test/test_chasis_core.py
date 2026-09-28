"""Tests de chasis_core. Corren en el PC con pytest, sin ROS.

El ultimo lee el xacro real y solo corre si hay xacro (con ROS cargado).
"""
import math
import os
import random

import pytest

from starcrawler_odometry.chasis_core import (
    Geometria,
    altura_pivote,
    geometria_desde_urdf,
    pose_chasis,
)

R1, R2, L = 0.0764, 0.0415, 0.36
GEO = Geometria([(0.30, -0.24), (0.30, 0.24), (-0.30, -0.24), (-0.30, 0.24)],
                R1, R2, L, R1)
g = math.radians


def pose(fr, fl, rr, rl):
    return pose_chasis([g(fr), g(fl), g(rr), g(rl)], GEO)


def apoyan(p):
    return [h < 1e-6 for h in p.holguras]


# --- Una oruga -----------------------------------------------------------

def test_en_llano_apoya_la_polea_activa():
    assert altura_pivote(0.0, GEO) == R1


def test_levantada_sigue_apoyando_la_polea_activa():
    assert altura_pivote(g(60), GEO) == R1


def test_bajada_apoya_la_punta():
    assert altura_pivote(g(-45), GEO) == pytest.approx(R2 + L * math.sin(g(45)))


# --- Poses simetricas ----------------------------------------------------

def test_en_llano_reposa_sin_inclinarse():
    p = pose(0, 0, 0, 0)
    assert p.altura == pytest.approx(R1)
    assert p.cabeceo == pytest.approx(0.0) and p.balanceo == pytest.approx(0.0)
    assert all(apoyan(p))


def test_con_todo_levantado_sigue_en_llano():
    assert pose(45, 45, 45, 45).altura == pytest.approx(R1)


def test_con_todo_bajado_se_pone_de_pie():
    p = pose(-45, -45, -45, -45)
    assert p.altura == pytest.approx(R2 + L * math.sin(g(45)))
    assert p.cabeceo == pytest.approx(0.0) and p.balanceo == pytest.approx(0.0)


def test_con_todo_vertical_hacia_abajo():
    assert pose(-90, -90, -90, -90).altura == pytest.approx(R2 + L)


# --- Cabeceo y balanceo --------------------------------------------------

def test_bajar_las_delanteras_levanta_el_morro():
    p = pose(-30, -30, 0, 0)
    assert p.cabeceo < 0.0              # REP-103: + es morro abajo
    assert p.balanceo == pytest.approx(0.0)
    assert all(apoyan(p))


def test_bajar_las_traseras_baja_el_morro():
    assert pose(0, 0, -30, -30).cabeceo > 0.0


def test_bajar_las_izquierdas_levanta_la_izquierda():
    p = pose(0, -30, 0, -30)
    assert p.balanceo > 0.0             # REP-103: + es lado izquierdo arriba
    assert p.cabeceo == pytest.approx(0.0)
    assert all(apoyan(p))


def test_la_pose_es_simetrica():
    a = pose(-30, -30, 10, 10)
    b = pose(10, 10, -30, -30)
    assert a.cabeceo == pytest.approx(-b.cabeceo)
    assert a.altura == pytest.approx(b.altura)


def test_el_cabeceo_cuenta_para_la_elevacion_vista_desde_el_suelo():
    """Delante +45 y detras -45: el morro baja hasta que las cuatro apoyan."""
    p = pose(45, 45, -45, -45)
    assert all(apoyan(p))
    # las traseras (x = -0,30) apoyan la punta con la elevacion corregida
    assert p.altura + 0.30 * math.sin(p.cabeceo) == pytest.approx(
        altura_pivote(g(-45) + p.cabeceo, GEO))


def test_una_sola_oruga_bajada_queda_sobre_una_diagonal():
    p = pose(-30, 0, 0, 0)
    assert apoyan(p) == [True, False, False, True]
    assert p.holguras[1] == pytest.approx(p.holguras[2])


def test_nunca_atraviesa_el_suelo():
    rnd = random.Random(7)
    for _ in range(2000):
        e = [rnd.uniform(-math.pi / 2, math.pi / 2) for _ in range(4)]
        p = pose_chasis(e, GEO)
        assert min(p.holguras) > -1e-9
        assert sum(apoyan(p)) >= 2


# --- Geometria desde el URDF ---------------------------------------------

def _urdf(poleas=2):
    juntas = ''
    for nombre, x, y in (('fr', 0.3, -0.24), ('fl', 0.3, 0.24),
                         ('rr', -0.3, -0.24), ('rl', -0.3, 0.24)):
        cil = ''.join(
            '<visual><origin xyz="%g 0 0"/><geometry><cylinder radius="%g" '
            'length="0.08"/></geometry></visual>' % (xx, r)
            for xx, r in ((0.0, R1), (x / 0.3 * L, R2))[:poleas])
        juntas += (
            '<joint name="crawler_%s_joint" type="revolute"><parent link="base_link"/>'
            '<child link="c_%s"/><origin xyz="%g %g 0"/></joint>'
            '<link name="c_%s">%s</link>' % (nombre, nombre, x, y, nombre, cil))
    return ('<robot name="r"><link name="base_footprint"/>'
            '<joint name="chassis_lift_joint" type="prismatic">'
            '<parent link="base_footprint"/><child link="base_link"/>'
            '<origin xyz="0 0 %g"/></joint><link name="base_link"/>%s</robot>'
            % (R1, juntas))


def test_lee_la_geometria_del_urdf():
    geo = geometria_desde_urdf(_urdf())
    assert geo.pivotes[0] == (0.3, -0.24) and geo.pivotes[3] == (-0.3, 0.24)
    assert (geo.radio_polea, geo.radio_punta) == (R1, R2)
    assert geo.largo == pytest.approx(L)
    assert geo.altura_reposo == pytest.approx(R1)


def test_sin_dos_poleas_es_un_error():
    with pytest.raises(ValueError):
        geometria_desde_urdf(_urdf(poleas=1))


XACRO = os.path.join(os.path.dirname(__file__), '..', '..',
                     'starcrawler_description', 'urdf', 'starcrawler.urdf.xacro')


def test_el_urdf_del_robot_da_la_geometria_y_reposa_en_cero():
    xacro = pytest.importorskip('xacro')
    geo = geometria_desde_urdf(xacro.process_file(XACRO).toxml())
    assert (geo.radio_polea, geo.radio_punta, geo.largo) == pytest.approx((R1, R2, L))
    # Con las orugas en llano las juntas virtuales valen cero
    assert pose_chasis([0.0] * 4, geo).altura == pytest.approx(geo.altura_reposo)
