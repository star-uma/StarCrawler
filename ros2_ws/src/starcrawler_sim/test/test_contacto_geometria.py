"""Tests de la geometria de contacto_core: la oruga como envolvente de sus dos
poleas, la clasificacion de las restricciones, la ley del primer contacto y la
lectura del URDF. Sin ROS, salvo el ultimo, que lee el xacro real."""
import math
import os

import pytest

from starcrawler_sim import contacto_core as cc
from terreno_prueba import BETA, GEO_PRUEBA, L, R1, R2

g = math.radians


def forma(q, cabeceo=0.0):
    return cc.forma_oruga(0, q, GEO_PRUEBA, cabeceo)


# --- La oruga ------------------------------------------------------------

def test_beta_es_el_belt_angle_del_xacro():
    # xacro: belt_angle = asin((pulley_radius - tip_radius) / arm_length)
    assert math.degrees(BETA) == pytest.approx(5.563, abs=5e-4)


def test_con_la_oruga_en_llano_soporta_la_polea_activa():
    s, z, polea, _ = cc.punto_soporte(forma(0.0), 0.0, -1.0)
    assert polea == 1
    assert (s, z) == pytest.approx((0.30, -R1), abs=1e-12)


def test_con_la_oruga_bajada_soporta_la_punta():
    s, z, polea, _ = cc.punto_soporte(forma(g(-10)), 0.0, -1.0)
    assert polea == 2
    assert (s, z) == pytest.approx((0.30 + L * math.cos(g(10)),
                                    -L * math.sin(g(10)) - R2), abs=1e-12)


def test_con_menos_beta_empatan_las_dos_poleas():
    f = forma(-BETA)[1]
    assert -f[1] + f[2] == pytest.approx(-f[4] + f[5], abs=1e-9)


def test_borde_inferior_en_llano():
    f = forma(0.0)
    assert cc.borde_inferior(f, 0.30) == pytest.approx(-R1)
    # Bajo el eje de la punta manda la tangente, inclinada beta
    assert cc.borde_inferior(f, 0.66) == pytest.approx(-R2 / math.cos(BETA))
    assert cc.borde_inferior(f, 0.66 + R2 - 1e-12) == pytest.approx(0.0, abs=1e-6)
    assert cc.borde_inferior(f, 0.30 - R1 - 1e-3) == math.inf


def test_con_el_brazo_vertical_abajo_solo_esta_la_polea_activa():
    f = forma(g(90))
    assert cc.borde_inferior(f, 0.30) == pytest.approx(-R1)
    assert cc.borde_inferior(f, 0.35) == pytest.approx(-math.sqrt(R1 ** 2 - 0.05 ** 2))


# --- Restricciones -------------------------------------------------------

def test_un_canto_sobre_el_tramo_inferior_con_30_grados():
    perfil = [(-1.0, 0.0), (0.45, 0.0), (0.45, 0.15), (1.0, 0.15)]
    rs = cc.restricciones_linea(perfil, [forma(g(30))], z=R1)
    canto = [r for r in rs if r.clave[2] == 'v' and r.z == 0.15]
    assert len(canto) == 1
    r = canto[0]
    assert math.degrees(math.acos(r.nz)) == pytest.approx(30 + math.degrees(BETA), abs=0.01)
    assert r.apoyo and r.traccion
    assert r.s == pytest.approx(0.45)


def test_la_cara_de_un_escalon_contra_la_punta_es_pared():
    # Canto a la altura del eje de la polea pasiva: 90 grados
    perfil = [(-1.0, 0.0), (0.69, 0.0), (0.69, R1), (1.0, R1)]
    r = [r for r in cc.restricciones_linea(perfil, [forma(0.0)], z=R1)
         if r.clave[2] == 'v' and r.z == R1][0]
    assert math.degrees(math.acos(r.nz)) == pytest.approx(90.0, abs=1e-6)
    assert not r.apoyo and r.empuje < 0.0        # empuja la base hacia atras


def test_un_canto_por_encima_del_eje_pasa_de_90_grados():
    perfil = [(-1.0, 0.0), (0.69, 0.0), (0.69, 0.5), (1.0, 0.5)]
    r = [r for r in cc.restricciones_linea(perfil, [forma(0.0)], z=R1)
         if r.clave[2] == 'v' and r.z == 0.5][0]
    assert r.nz < 0.0 and not r.apoyo
    # Lo que hay que retroceder para que la polea deje de tocarlo
    assert r.empuje == pytest.approx(-(0.66 + R2 - 0.69), abs=1e-5)


def test_las_rampas_lisas_de_mas_de_40_grados_no_traccionan():
    for grados, traccion in ((30.0, True), (45.0, False)):
        m = math.tan(g(grados))
        perfil = [(-1.0, -m), (1.0, m)]
        rs = cc.restricciones_linea(perfil, [forma(0.0)], z=R1)
        assert rs and all(r.traccion == traccion for r in rs if r.clave[2] == 't')


def test_el_chasis_nunca_tracciona_y_apoya_hasta_60_grados():
    perfil = [(-1.0, 0.0), (-0.05, 0.0), (-0.05, 0.12), (0.05, 0.12), (0.05, 0.0), (1.0, 0.0)]
    rs = cc.restricciones_linea(perfil, [cc.forma_chasis(GEO_PRUEBA)], z=0.17)
    arriba = [r for r in rs if r.z == 0.12]
    assert arriba and all(r.apoyo and not r.traccion for r in arriba)
    assert max(r.altura for r in rs) == pytest.approx(0.17)


# --- Ley del primer contacto (§5.6) ----------------------------------------

@pytest.mark.parametrize('q, h', [
    (0, 0.0657), (10, 0.1282), (20, 0.1888), (30, 0.2457), (45, 0.3202),
    (60, 0.3774), (69, 0.4017), (70, 0.0566), (80, 0.0566), (-30, 0.0308)])
def test_altura_trepable(q, h):
    assert cc.altura_trepable(g(q), GEO_PRUEBA) == pytest.approx(h, abs=5e-5)


def test_el_maximo_trepable_esta_en_75_menos_beta():
    """El maximo esta donde el tramo inferior llega a 75 grados, en 69,44:
    0,4027 (la tabla de §5.6 da 0,4017, que es el valor en 69)."""
    alturas = [(cc.altura_trepable(g(k / 100.0), GEO_PRUEBA), k / 100.0)
               for k in range(0, 9000)]
    h, q = max(alturas)
    assert q == pytest.approx(75.0 - math.degrees(BETA), abs=0.01)
    assert h == pytest.approx(0.4027, abs=5e-5)


# --- URDF ----------------------------------------------------------------

def _urdf(caja=True, masa_brazo=3.0):
    juntas = ''
    for nombre, x, y in (('fr', 0.3, -0.24), ('fl', 0.3, 0.24),
                         ('rr', -0.3, -0.24), ('rl', -0.3, 0.24)):
        sig = 1 if x > 0 else -1
        cil = ''.join(
            '<visual><origin xyz="%g 0 0"/><geometry><cylinder radius="%g" '
            'length="0.08"/></geometry></visual>' % (xx, r)
            for xx, r in ((0.0, R1), (sig * L, R2)))
        juntas += (
            '<joint name="crawler_%s_joint" type="revolute"><parent link="base_link"/>'
            '<child link="c_%s"/><origin xyz="%g %g 0"/></joint>'
            '<link name="c_%s">%s<inertial><origin xyz="%g 0 0"/><mass value="%g"/>'
            '</inertial></link>' % (nombre, nombre, x, y, nombre, cil, sig * L / 2,
                                    masa_brazo if nombre != 'rl' else 3.0))
    visual = ('<visual><geometry><box size="0.6 0.4 0.1"/></geometry></visual>'
              if caja else '')
    return ('<robot name="r"><link name="base_footprint"/>'
            '<joint name="chassis_lift_joint" type="prismatic">'
            '<parent link="base_footprint"/><child link="base_link"/>'
            '<origin xyz="0 0 %g"/></joint><link name="base_link">%s'
            '<inertial><mass value="20"/></inertial></link>%s</robot>'
            % (R1, visual, juntas))


def test_lee_la_geometria_del_robot_del_urdf():
    geo = cc.geometria_robot_desde_urdf(_urdf())
    assert geo.chasis == pytest.approx((0.60, 0.40, 0.10))
    assert geo.ancho_oruga == pytest.approx(0.08)
    assert (geo.masa_chasis, geo.masa_brazo) == (20.0, 3.0)
    assert geo.orugas.radio_polea == R1 and geo.orugas.largo == pytest.approx(L)


def test_sin_caja_del_chasis_es_un_error():
    with pytest.raises(ValueError):
        cc.geometria_robot_desde_urdf(_urdf(caja=False))


def test_brazos_de_distinta_masa_son_un_error():
    with pytest.raises(ValueError):
        cc.geometria_robot_desde_urdf(_urdf(masa_brazo=4.0))


XACRO = os.path.join(os.path.dirname(__file__), '..', '..',
                     'starcrawler_description', 'urdf', 'starcrawler.urdf.xacro')


def test_el_xacro_del_robot_da_la_geometria_de_prueba():
    xacro = pytest.importorskip('xacro')
    geo = cc.geometria_robot_desde_urdf(xacro.process_file(XACRO).toxml())
    assert geo.chasis == pytest.approx(GEO_PRUEBA.chasis)
    assert geo.ancho_oruga == pytest.approx(0.08)
    assert (geo.masa_chasis, geo.masa_brazo) == pytest.approx((20.0, 3.0))
    o, p = geo.orugas, GEO_PRUEBA.orugas
    assert (o.radio_polea, o.radio_punta, o.largo, o.altura_reposo) == pytest.approx(
        (p.radio_polea, p.radio_punta, p.largo, p.altura_reposo))
    assert [c for v in o.pivotes for c in v] == pytest.approx(
        [c for v in p.pivotes for c in v])
