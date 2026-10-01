"""Los tacos de las orugas: periodos del URDF y avance modulo un paso."""

import pytest

from starcrawler_odometry.bandas_core import Bandas, juntas_de_tacos

URDF = """<robot name="r">
  <joint name="crawler_fr_tacos_abajo_joint" type="prismatic">
    <limit lower="0" upper="0.03" effort="0" velocity="1"/></joint>
  <joint name="crawler_fr_tacos_polea_joint" type="revolute">
    <limit lower="0" upper="0.4" effort="0" velocity="1"/></joint>
  <joint name="crawler_fl_tacos_abajo_joint" type="prismatic">
    <limit lower="0" upper="0.03" effort="0" velocity="1"/></joint>
  <joint name="crawler_fr_joint" type="revolute">
    <limit lower="-1.5" upper="1.5" effort="0" velocity="1"/></joint>
</robot>"""


def test_lee_las_juntas_de_tacos_y_su_periodo():
    j = juntas_de_tacos(URDF)
    assert j['crawler_fr'] == [('crawler_fr_tacos_abajo_joint', 0.03),
                               ('crawler_fr_tacos_polea_joint', 0.4)]
    assert j['crawler_fl'] == [('crawler_fl_tacos_abajo_joint', 0.03)]
    assert j['crawler_rr'] == []


def test_avance_modulo_un_paso_en_tramos_y_arcos():
    b = Bandas(juntas_de_tacos(URDF))
    b.avanzar(0.0, 0.045, 1.0)              # derecha: 1,5 pasos
    pos = dict(zip(*b.posiciones()))
    assert pos['crawler_fr_tacos_abajo_joint'] == pytest.approx(0.015)
    assert pos['crawler_fr_tacos_polea_joint'] == pytest.approx(0.2)
    assert pos['crawler_fl_tacos_abajo_joint'] == pytest.approx(0.0)


def test_marcha_atras_queda_dentro_del_periodo():
    b = Bandas(juntas_de_tacos(URDF))
    b.avanzar(-0.01, -0.01, 1.0)
    for valor in b.posiciones()[1]:
        assert 0.0 <= valor < 0.4
    assert dict(zip(*b.posiciones()))['crawler_fl_tacos_abajo_joint'] == pytest.approx(0.02)
