"""El SDF de Gazebo: XML valido, ruedas en su lado, rampas como cajas giradas."""

import math
import xml.etree.ElementTree as ET

from starcrawler_gazebo import sdf_core
from starcrawler_odometry.chasis_core import Geometria
from starcrawler_sim import mundo_core
from starcrawler_sim.contacto_core import GeometriaRobot

PIVOTES = [(0.30, -0.24), (0.30, 0.24), (-0.30, -0.24), (-0.30, 0.24)]
GEO = GeometriaRobot(Geometria(PIVOTES, 0.0764, 0.0415, 0.36, 0.0764),
                     (0.60, 0.40, 0.10), 0.08, 20.0, 3.0)


def mundo(nombre):
    return mundo_core.cargar_fichero(mundo_core.resolver_ruta(nombre))


def test_ruedas_tocan_la_banda_y_se_solapan():
    lista = sdf_core.ruedas(1.0, 0.0764, 0.0415, 0.36)
    assert lista[0] == (0.0, 0.0, 0.0764, 'polea')
    assert lista[1] == (0.36, 0.0, 0.0415, 'punta')
    rodillos = sorted([r for r in lista if r[3] == 'rodillo' and r[1] < 0])
    assert len(rodillos) >= 5
    for a, b in zip(rodillos, rodillos[1:]):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) < 2 * a[2]   # solapados


def test_cada_rueda_escucha_a_su_lado(tmp_path):
    raiz = ET.fromstring(sdf_core.mundo_sdf(mundo('llano'), GEO, str(tmp_path)))
    for plugin in raiz.iter('plugin'):
        junta = plugin.findtext('joint_name') or ''
        topico = plugin.findtext('topic') or ''
        if junta.endswith('_j'):
            lado = 'izq' if ('crawler_fl' in junta or 'crawler_rl' in junta) else 'der'
            assert '/banda_%s/' % lado in topico, (junta, topico)


def test_topicos_de_ros_validos():
    for p in sdf_core.puentes():
        topico = p.split('@')[0]
        assert all(not t[:1].isdigit() for t in topico.split('/') if t), topico


def test_la_rampa_es_una_caja_con_el_techo_en_su_plano(tmp_path):
    m = mundo('rampa')
    texto = sdf_core.mundo_sdf(m, GEO, str(tmp_path))
    assert '<mesh>' not in texto            # el ODE de DART se cae con mallas
    raiz = ET.fromstring(texto)
    terreno = [x for x in raiz.iter('model') if x.get('name') == 'terreno'][0]
    inclinados = [so for so in m.solidos if so.bx or so.by]
    assert inclinados
    for so in inclinados:
        k = m.solidos.index(so)
        eslabon = [e for e in terreno.iter('link') if e.get('name') == 's%d' % k][0]
        x, y, z, rol, cab, rumbo = (float(v) for v in eslabon.findtext('pose').split())
        grosor = float(eslabon.find('.//box/size').text.split()[2])
        # Centro de la cara de arriba: centro + R(rol, cab, rumbo) (0, 0, grosor/2)
        h = grosor / 2
        cz = math.cos(cab) * math.cos(rol)
        ux = math.cos(rumbo) * math.sin(cab) * math.cos(rol) + math.sin(rumbo) * math.sin(rol)
        uy = math.sin(rumbo) * math.sin(cab) * math.cos(rol) - math.cos(rumbo) * math.sin(rol)
        tx, ty, tz = x + h * ux, y + h * uy, z + h * cz
        assert abs(so.techo(tx, ty) - tz) < 0.005, (so.origen, tz, so.techo(tx, ty))


def test_la_pista_completa_genera(tmp_path):
    texto = sdf_core.mundo_sdf(mundo('practica_rrl'), GEO, str(tmp_path))
    ET.fromstring(texto)
    assert texto.count('<link name="s') > 100
