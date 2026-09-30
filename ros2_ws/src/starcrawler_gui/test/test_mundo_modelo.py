"""Tests de la traduccion del mundo para la vista 3D. Corren en el PC, sin ROS.

Los Marker se imitan con SimpleNamespace. El ultimo test usa el mensaje de
verdad y solo corre con visualization_msgs a mano (con ROS cargado).
"""
import json
import math
from types import SimpleNamespace as NS

import pytest

from starcrawler_gui.mundo_modelo import (
    ADD,
    CLAVES_ESTADO,
    DELETE,
    DELETEALL,
    MAX_ESTADO,
    TIPOS,
    Traductor,
    traducir,
    traducir_lista,
    validar_estado,
)

TRIANGLE_LIST, LINE_LIST, LINE_STRIP, SPHERE_LIST = 11, 5, 4, 7
ARROW, CUBE, TEXT, MESH = 0, 1, 9, 10


def punto(x, y, z):
    return NS(x=x, y=y, z=z)


def rgba(r, g, b, a=1.0):
    return NS(r=r, g=g, b=b, a=a)


def marker(ns='terreno', id=0, tipo=TRIANGLE_LIST, accion=ADD, marco='odom',
           puntos=(), colores=(), color=(1.0, 1.0, 1.0, 1.0),
           pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0),
           escala=(1.0, 1.0, 1.0), texto=''):
    return NS(
        header=NS(frame_id=marco), ns=ns, id=id, type=tipo, action=accion,
        pose=NS(position=punto(*pos),
                orientation=NS(x=quat[0], y=quat[1], z=quat[2], w=quat[3])),
        scale=NS(x=escala[0], y=escala[1], z=escala[2]),
        color=rgba(*color),
        points=[punto(*p) for p in puntos],
        colors=[rgba(*c) for c in colores],
        text=texto)


TRIANGULO = ((0.0, 0.0, 0.2), (1.0, 0.0, 0.2), (0.0, 1.0, 0.2))


def claves(traductor):
    return [p['clave'] for p in traductor.a_json()['piezas']]


# ─── Tipos ───────────────────────────────────────────────────────────────────

def test_los_tipos_son_los_del_contrato_de_visual():
    assert sorted(TIPOS.values()) == sorted(
        ['triangulos', 'lineas', 'tira', 'esferas', 'flecha', 'texto'])


def test_un_cube_u_otro_tipo_raro_va_a_omitidos():
    t = Traductor()
    t.aplicar([marker(ns='a', tipo=CUBE), marker(ns='b', tipo=MESH),
               marker(ns='c', tipo=99)])
    j = t.a_json()
    assert j['piezas'] == [] and j['omitidos'] == 3


def test_triangle_list_con_colores_por_vertice():
    m = marker(puntos=TRIANGULO,
               colores=[(0.42, 0.40, 0.37, 1.0)] * 2 + [(0.8, 0.78, 0.71, 1.0)])
    p = traducir(m)
    assert p['tipo'] == 'triangulos' and p['clave'] == 'terreno/0'
    assert p['puntos'] == [0.0, 0.0, 0.2, 1.0, 0.0, 0.2, 0.0, 1.0, 0.2]
    assert p['colores'] == [0.42, 0.4, 0.37, 1.0] * 2 + [0.8, 0.78, 0.71, 1.0]
    assert len(p['colores']) == 4 * len(p['puntos']) // 3


def test_un_color_por_cara_se_reparte_a_sus_vertices():
    p = traducir(marker(puntos=TRIANGULO, colores=[(1.0, 0.0, 0.0, 1.0)]))
    assert p['colores'] == [1.0, 0.0, 0.0, 1.0] * 3


def test_sin_colores_o_con_otra_cuenta_no_hay_colores():
    assert traducir(marker(puntos=TRIANGULO))['colores'] is None
    dos = [(1.0, 0.0, 0.0, 1.0)] * 2
    assert traducir(marker(puntos=TRIANGULO, colores=dos))['colores'] is None


def test_la_pieza_tiene_todas_las_claves():
    p = traducir(marker(tipo=TEXT, pos=(1.0, 2.0, 0.4), texto='Carril 1',
                        escala=(0.0, 0.0, 0.08)))
    assert set(p) == {'clave', 'tipo', 'pos', 'quat', 'escala', 'color',
                      'puntos', 'colores', 'texto'}
    assert p['tipo'] == 'texto' and p['texto'] == 'Carril 1'
    assert p['pos'] == [1.0, 2.0, 0.4] and p['escala'][2] == 0.08
    json.dumps(p, allow_nan=False)


def test_solo_los_textos_llevan_texto():
    assert traducir(marker(puntos=TRIANGULO, texto='x'))['texto'] == ''


def test_cuentas_de_puntos_imposibles_van_a_omitidos():
    """RViz tampoco dibuja un TRIANGLE_LIST sin multiplo de 3."""
    assert traducir(marker(puntos=TRIANGULO[:2])) is None
    assert traducir(marker(tipo=LINE_LIST, puntos=TRIANGULO)) is None
    assert traducir(marker(tipo=LINE_LIST, puntos=TRIANGULO[:2])) is not None
    assert traducir(marker(tipo=LINE_STRIP, puntos=TRIANGULO)) is not None


def test_la_flecha_necesita_dos_puntos():
    assert traducir(marker(tipo=ARROW)) is None
    f = traducir(marker(tipo=ARROW, puntos=TRIANGULO[:2],
                        escala=(0.02, 0.04, 0.05)))
    assert f['tipo'] == 'flecha' and len(f['puntos']) == 6
    assert f['escala'] == [0.02, 0.04, 0.05]


def test_esferas_con_color_por_punto():
    p = traducir(marker(tipo=SPHERE_LIST, puntos=TRIANGULO[:2],
                        colores=[(1, 1, 1, 1), (0.98, 0.7, 0.1, 1)],
                        escala=(0.035, 0.035, 0.035)))
    assert p['tipo'] == 'esferas' and len(p['colores']) == 8


def test_un_frame_distinto_va_a_omitidos():
    t = Traductor('odom')
    t.aplicar([marker(marco='map', puntos=TRIANGULO),
               marker(ns='b', marco='', puntos=TRIANGULO),
               marker(ns='c', marco='/odom', puntos=TRIANGULO)])
    assert claves(t) == ['c/0'] and t.a_json()['omitidos'] == 2


def test_el_marco_fijo_se_puede_cambiar():
    assert traducir_lista([marker(marco='map', puntos=TRIANGULO)], 'map')


def test_numeros_no_finitos_van_a_omitidos():
    """Un NaN en el JSON rompe el JSON.parse de la pagina."""
    malo = ((0.0, float('nan'), 0.0),) + TRIANGULO[1:]
    assert traducir(marker(puntos=malo)) is None
    assert traducir(marker(puntos=TRIANGULO, pos=(math.inf, 0, 0))) is None
    assert traducir(marker(puntos=TRIANGULO,
                           colores=[(0, 0, float('nan'), 1)] * 3)) is None


def test_el_redondeo():
    p = traducir(marker(puntos=((0.12345, -0.0006, 1.0004),) + TRIANGULO[1:],
                        colores=[(0.12345, 0.9996, 0.5, 0.28)] * 3,
                        color=(0.4781, 0.51, 0.549, 0.2804)))
    assert p['puntos'][:3] == [0.123, -0.001, 1.0]
    assert p['colores'][:4] == [0.123, 1.0, 0.5, 0.28]
    assert p['color'] == [0.478, 0.51, 0.549, 0.28]


def test_el_cuaternion_nulo_es_la_identidad():
    """Como en RViz; y los demas se normalizan."""
    assert traducir(marker(puntos=TRIANGULO,
                           quat=(0, 0, 0, 0)))['quat'] == [0, 0, 0, 1]
    q = traducir(marker(puntos=TRIANGULO, quat=(0, 0, 2, 2)))['quat']
    assert q == pytest.approx([0, 0, math.sqrt(0.5), math.sqrt(0.5)])


# ─── Acciones y orden ────────────────────────────────────────────────────────

def test_deleteall_vacia():
    t = Traductor()
    t.aplicar([marker(ns='a', puntos=TRIANGULO), marker(ns='b', tipo=CUBE)])
    t.aplicar([marker(accion=DELETEALL)])
    assert t.a_json() == {'version': 2, 'piezas': [], 'omitidos': 0}


def test_delete_borra_solo_su_ns_id():
    t = Traductor()
    t.aplicar([marker(ns='a', id=0, puntos=TRIANGULO),
               marker(ns='a', id=1, puntos=TRIANGULO),
               marker(ns='b', id=0, puntos=TRIANGULO)])
    t.aplicar([marker(ns='a', id=0, accion=DELETE)])
    assert claves(t) == ['a/1', 'b/0']


def test_se_respeta_el_orden_dentro_del_array():
    t = Traductor()
    t.aplicar([marker(ns='viejo', puntos=TRIANGULO),
               marker(accion=DELETEALL),
               marker(ns='nuevo', puntos=TRIANGULO)])
    assert claves(t) == ['nuevo/0']
    t.aplicar([marker(ns='nuevo', accion=DELETE),
               marker(ns='nuevo', puntos=TRIANGULO, color=(1, 0, 0, 1))])
    assert claves(t) == ['nuevo/0']
    assert t.a_json()['piezas'][0]['color'] == [1.0, 0.0, 0.0, 1.0]


def test_modify_sustituye_y_una_pieza_mala_deja_su_hueco():
    t = Traductor()
    t.aplicar([marker(puntos=TRIANGULO)])
    t.aplicar([marker(tipo=CUBE)])
    assert t.a_json()['piezas'] == [] and t.a_json()['omitidos'] == 1
    t.aplicar([marker(puntos=TRIANGULO)])
    assert claves(t) == ['terreno/0'] and t.a_json()['omitidos'] == 0


def test_una_accion_desconocida_va_a_omitidos():
    t = Traductor()
    t.aplicar([marker(accion=7, puntos=TRIANGULO)])
    assert t.a_json()['omitidos'] == 1


def test_la_version_sube_con_cada_array():
    t = Traductor()
    assert t.a_json()['version'] == 0
    t.aplicar([])
    t.aplicar([marker(accion=DELETEALL)])
    assert t.a_json()['version'] == 2


def test_traducir_lista_no_guarda_estado():
    marcas = [marker(accion=DELETEALL),
              marker(ns='cdg', tipo=SPHERE_LIST, puntos=TRIANGULO[:1]),
              marker(ns='raro', tipo=CUBE)]
    piezas = traducir_lista(marcas)
    assert [p['clave'] for p in piezas] == ['cdg/0']
    assert traducir_lista([]) == []


def test_el_json_de_un_mundo_se_puede_servir():
    t = Traductor()
    t.aplicar([marker(accion=DELETEALL),
               marker(puntos=TRIANGULO, colores=[(0.5, 0.5, 0.5, 1)] * 3),
               marker(ns='aristas', tipo=LINE_LIST, puntos=TRIANGULO[:2],
                      color=(0.11, 0.11, 0.1, 0.7), escala=(0.004, 0, 0))])
    json.loads(json.dumps(t.a_json(), allow_nan=False))


# ─── /mundo/estado ───────────────────────────────────────────────────────────

ESTADO = {
    'v': 1, 'mundo': 'escalon', 'version': 3, 'modo': 'normal',
    'estado': 'bloqueado', 'pieza': 'FR',
    'motivo': 'el morro choca con un canto de 20 cm',
    'consejo': 'sube las delanteras (L1 · ▲ delanteras)',
    'apoya': [True, True, True, False], 'holgura': [0, 0, 0, 0.034],
    'panza': False, 'margen': 0.12, 'cabeceo': 0.01, 'balanceo': -0.02,
    'altura': 0.0764, 'z_suelo': 0.0, 'avance_orugas': 0.053,
    'avance_real': 0.0, 'patinado': 0.4, 'ms': 1.8,
}


def test_el_estado_bueno_pasa_entero():
    d = validar_estado(json.dumps(ESTADO))
    assert set(d) == set(ESTADO) == CLAVES_ESTADO
    assert d['estado'] == 'bloqueado' and d['apoya'][3] is False
    assert d['holgura'][3] == 0.034


def test_las_claves_desconocidas_se_quitan():
    d = validar_estado(json.dumps(dict(ESTADO, extra=[1, 2, 3])))
    assert 'extra' not in d


def test_los_no_finitos_quedan_en_null():
    """json.loads acepta NaN, y en /events romperia la pagina."""
    texto = json.dumps(dict(ESTADO, margen=float('inf'),
                            holgura=[0, float('nan'), 0, 0]))
    d = validar_estado(texto)
    assert d['margen'] is None and d['holgura'][1] is None
    json.dumps(d, allow_nan=False)


@pytest.mark.parametrize('malo', [
    'no es json', '[1, 2]', '{"estado": "libre"}',
    json.dumps(dict(ESTADO, v=2)), json.dumps(dict(ESTADO, v=True)),
    json.dumps(dict(ESTADO, cabeceo='mucho')),
    json.dumps(dict(ESTADO, cabeceo=True)),
    json.dumps(dict(ESTADO, estado=3)),
    json.dumps(dict(ESTADO, apoya=[True, False])),
    json.dumps(dict(ESTADO, apoya=[1, 1, 1, 1])),
    json.dumps(dict(ESTADO, panza='no')),
    json.dumps(dict(ESTADO, altura=10 ** 400)),
])
def test_lo_que_llega_mal_da_valueerror(malo):
    with pytest.raises(ValueError):
        validar_estado(malo)


def test_mas_de_4_kb_da_valueerror():
    largo = json.dumps(dict(ESTADO, motivo='x' * MAX_ESTADO))
    with pytest.raises(ValueError, match='bytes'):
        validar_estado(largo)


def test_null_vale_en_cualquier_clave():
    nulos = {k: None for k in CLAVES_ESTADO}
    nulos['v'] = 1
    d = validar_estado(json.dumps(nulos))
    assert d['pieza'] is None and d['apoya'] is None


# ─── Con el mensaje de ROS de verdad ────────────────────────────────────────

def test_con_el_marker_de_ros():
    vm = pytest.importorskip('visualization_msgs.msg')
    gm = pytest.importorskip('geometry_msgs.msg')
    sm = pytest.importorskip('std_msgs.msg')
    Marker = vm.Marker
    assert (Marker.TRIANGLE_LIST, Marker.LINE_LIST, Marker.LINE_STRIP,
            Marker.SPHERE_LIST, Marker.ARROW, Marker.TEXT_VIEW_FACING) == \
        (11, 5, 4, 7, 0, 9)
    assert (Marker.ADD, Marker.MODIFY, Marker.DELETE, Marker.DELETEALL) == \
        (ADD, ADD, DELETE, DELETEALL)

    borrar = Marker(action=Marker.DELETEALL)
    m = Marker(ns='terreno', id=0, type=Marker.TRIANGLE_LIST,
               action=Marker.ADD)
    m.header.frame_id = 'odom'
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = sm.ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    m.points = [gm.Point(x=x, y=y, z=z) for x, y, z in TRIANGULO]
    m.colors = [sm.ColorRGBA(r=0.42, g=0.4, b=0.37, a=1.0)] * 3
    texto = Marker(ns='etiquetas', id=0, type=Marker.TEXT_VIEW_FACING,
                   text='Carril 1')
    texto.header.frame_id = 'odom'
    texto.pose.position.z = 0.4
    raro = Marker(ns='raro', id=0, type=Marker.CUBE)
    raro.header.frame_id = 'odom'
    arr = vm.MarkerArray(markers=[borrar, m, texto, raro])

    t = Traductor()
    t.aplicar(arr.markers)
    j = t.a_json()
    assert [p['clave'] for p in j['piezas']] == ['terreno/0', 'etiquetas/0']
    assert j['omitidos'] == 1
    assert j['piezas'][0]['colores'][:4] == [0.42, 0.4, 0.37, 1.0]
    assert j['piezas'][1]['texto'] == 'Carril 1'
    assert j['piezas'][1]['pos'] == pytest.approx([0.0, 0.0, 0.4])
    json.dumps(j, allow_nan=False)
