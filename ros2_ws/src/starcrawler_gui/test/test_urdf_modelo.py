"""Tests de la lectura del URDF para la vista 3D. Corren en el PC, sin ROS.

El ultimo usa el xacro real del robot y solo corre si hay xacro instalado
(con ROS cargado lo hay; en un Python pelado se salta).
"""
import os

import pytest

from starcrawler_gui.urdf_modelo import leer_urdf

URDF_MINIMO = """<?xml version="1.0"?>
<robot name="prueba">
  <material name="rojo"><color rgba="1 0 0 1"/></material>
  <link name="base"/>
  <joint name="fijo" type="fixed">
    <parent link="base"/><child link="cuerpo"/>
    <origin xyz="0 0 0.1"/>
  </joint>
  <link name="cuerpo">
    <visual>
      <geometry><box size="0.6 0.4 0.1"/></geometry>
      <material name="rojo"/>
    </visual>
  </link>
  <joint name="brazo_joint" type="revolute">
    <parent link="cuerpo"/><child link="brazo"/>
    <origin xyz="0.3 0.2 0" rpy="0 0 1.5"/>
    <axis xyz="0 -1 0"/>
  </joint>
  <link name="brazo">
    <visual>
      <origin xyz="0.18 0 0"/>
      <geometry><cylinder radius="0.05" length="0.36"/></geometry>
      <material name="propio"><color rgba="0 0 1 0.5"/></material>
    </visual>
    <visual>
      <geometry><mesh filename="package://x/brazo.stl"/></geometry>
    </visual>
  </link>
  <joint name="rueda_joint" type="continuous">
    <parent link="brazo"/><child link="rueda"/>
  </joint>
  <link name="rueda">
    <visual><geometry><sphere radius="0.04"/></geometry></visual>
  </link>
</robot>
"""


def modelo():
    return leer_urdf(URDF_MINIMO)


def test_la_raiz_es_el_link_que_no_es_hijo_de_nadie():
    assert modelo()['raiz'] == 'base'


def test_un_link_sin_visual_queda_vacio():
    assert modelo()['links']['base'] == []


def test_caja_con_material_por_nombre():
    (caja,) = modelo()['links']['cuerpo']
    assert caja['tipo'] == 'caja'
    assert caja['tam'] == [0.6, 0.4, 0.1]
    assert caja['color'] == [1.0, 0.0, 0.0, 1.0]


def test_cilindro_con_color_propio_y_origen():
    (cil,) = modelo()['links']['brazo']
    assert cil['tipo'] == 'cilindro'
    assert cil['radio'] == 0.05 and cil['largo'] == 0.36
    assert cil['color'] == [0.0, 0.0, 1.0, 0.5]
    assert cil['origen']['xyz'] == [0.18, 0.0, 0.0]


def test_las_mallas_se_cuentan_pero_no_se_dibujan():
    assert modelo()['mallas_omitidas'] == 1


def test_sin_material_sale_gris():
    (esf,) = modelo()['links']['rueda']
    assert esf['tipo'] == 'esfera'
    assert esf['color'] == [0.6, 0.6, 0.6, 1.0]


def test_joint_con_origen_y_eje():
    j = {x['nombre']: x for x in modelo()['joints']}['brazo_joint']
    assert j['tipo'] == 'revolute'
    assert (j['padre'], j['hijo']) == ('cuerpo', 'brazo')
    assert j['origen'] == {'xyz': [0.3, 0.2, 0.0], 'rpy': [0.0, 0.0, 1.5]}
    assert j['eje'] == [0.0, -1.0, 0.0]


def test_joint_sin_origen_ni_eje_toma_los_valores_del_estandar():
    """URDF: origen en cero y eje X si no se declaran."""
    j = {x['nombre']: x for x in modelo()['joints']}['rueda_joint']
    assert j['origen'] == {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]}
    assert j['eje'] == [1.0, 0.0, 0.0]


def test_dos_raices_es_un_error():
    doble = '<robot name="x"><link name="a"/><link name="b"/></robot>'
    with pytest.raises(ValueError):
        leer_urdf(doble)


# --- El URDF real del robot ---------------------------------------------

XACRO = os.path.join(os.path.dirname(__file__), '..', '..',
                     'starcrawler_description', 'urdf',
                     'starcrawler.urdf.xacro')


def test_el_urdf_del_robot_tiene_lo_que_dibuja_la_vista():
    xacro = pytest.importorskip('xacro')
    m = leer_urdf(xacro.process_file(XACRO).toxml())

    # La odometria publica odom -> base_footprint: tiene que ser la raiz
    assert m['raiz'] == 'base_footprint'
    assert m['mallas_omitidas'] == 0

    juntas = {j['nombre']: j for j in m['joints']}
    for nombre in ('crawler_fr_joint', 'crawler_fl_joint',
                   'crawler_rr_joint', 'crawler_rl_joint'):
        assert juntas[nombre]['tipo'] == 'revolute'
        assert m['links'][juntas[nombre]['hijo']], nombre + ' sin visual'

    # Positivo = brazo levantado: delanteras giran sobre -Y, traseras sobre +Y
    assert juntas['crawler_fr_joint']['eje'] == [0.0, -1.0, 0.0]
    assert juntas['crawler_rl_joint']['eje'] == [0.0, 1.0, 0.0]
