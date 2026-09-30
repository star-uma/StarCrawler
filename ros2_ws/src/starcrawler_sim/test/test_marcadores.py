"""Tests de marcadores. Con un Estado falso: no necesitan ROS ni contacto_core,
salvo la conversion a Marker, que se salta sin visualization_msgs."""
import json
import math
from types import SimpleNamespace

import pytest

from starcrawler_sim import marcadores as mk

CLAVES_ESTADO = {
    'v', 'mundo', 'version', 'modo', 'estado', 'pieza', 'motivo', 'consejo',
    'apoya', 'holgura', 'panza', 'margen', 'cabeceo', 'balanceo', 'altura',
    'z_suelo', 'avance_orugas', 'avance_real', 'patinado', 'ms'}


def contacto(pieza='FR', x=0.3, y=-0.24, z=0.0, normal=(0.0, 0.0, 1.0),
             tipo='apoyo', holgura=0.0):
    return SimpleNamespace(pieza=pieza, x=x, y=y, z=z, normal=normal,
                           ataque=0.0, tipo=tipo, traccion=tipo == 'apoyo',
                           holgura=holgura)


def estado(**cambios):
    """El robot en reposo en llano, apoyado en las cuatro poleas."""
    base = dict(
        pose=SimpleNamespace(x=1.0, y=2.0, yaw=0.5, z=0.0764,
                             cabeceo=0.0, balanceo=0.0),
        elevaciones=(0.0, 0.0, 0.0, 0.0),
        contactos=tuple(contacto(p, x, y) for p, x, y in (
            ('FR', 0.3, -0.24), ('FL', 0.3, 0.24),
            ('RR', -0.3, -0.24), ('RL', -0.3, 0.24))),
        apoya=(True, True, True, True), holguras=(0.0, 0.0, 0.0, 0.0),
        panza=False, bloqueado=None, sin_traccion=False, cayendo=False,
        volcado=False, atrapado=False, cdg=(1.0, 2.0, 0.08),
        poligono=((1.3, 1.76, 0.0), (1.3, 2.24, 0.0),
                  (0.7, 2.24, 0.0), (0.7, 1.76, 0.0)),
        margen=0.24, avance=0.0, propuesto=0.0)
    base.update(cambios)
    return SimpleNamespace(**base)


def contra_escalon(pieza='FR'):
    """Bloqueado por la polea pasiva contra la cara de un escalon en +x."""
    cara = contacto(pieza, x=1.0, y=0.0, z=0.0764, normal=(-1.0, 0.0, 0.0),
                    tipo='pared', holgura=-0.001)
    return estado(bloqueado=pieza, contactos=estado().contactos + (cara,))


def leer(texto):
    def prohibido(c):
        raise ValueError('JSON no estandar: %s' % c)
    return json.loads(texto, parse_constant=prohibido)


def por_ns(piezas):
    return {p['ns']: p for p in piezas}


# --- Clasificacion y textos -----------------------------------------------

@pytest.mark.parametrize('activos, esperado', [
    (('volcado', 'atrapado', 'cayendo', 'bloqueado', 'sin_traccion'), 'volcado'),
    (('atrapado', 'cayendo', 'bloqueado', 'sin_traccion'), 'atrapado'),
    (('cayendo', 'bloqueado', 'sin_traccion'), 'cayendo'),
    (('bloqueado', 'sin_traccion'), 'bloqueado'),
    (('sin_traccion',), 'sin_traccion'),
    ((), 'libre'),
])
def test_prioridad_de_los_estados(activos, esperado):
    banderas = {c: c in activos for c in mk.PRIORIDAD}
    if banderas['bloqueado']:
        banderas['bloqueado'] = 'FR'
    else:
        banderas['bloqueado'] = None
    e = estado(**banderas)
    assert mk.clasificar(e) == esperado
    assert leer(mk.estado_json(e))['estado'] == esperado


def test_libre_trae_todas_las_claves_y_sin_textos():
    d = leer(mk.estado_json(estado(), mundo='escalon', version=3))
    assert set(d) == CLAVES_ESTADO
    assert d['v'] == 1 and d['mundo'] == 'escalon' and d['version'] == 3
    assert d['modo'] == 'normal' and d['estado'] == 'libre'
    assert d['pieza'] is None and d['motivo'] == '' and d['consejo'] == ''
    assert d['apoya'] == [True] * 4 and d['holgura'] == [0.0] * 4
    assert d['altura'] == pytest.approx(0.0764)


def test_bloqueado_por_FR_aconseja_L1_con_el_alto_del_canto():
    d = leer(mk.estado_json(contra_escalon('FR'), alto_choque=0.20))
    assert d['estado'] == 'bloqueado' and d['pieza'] == 'FR'
    assert d['motivo'] == 'el morro choca con un canto de 20 cm'
    assert d['consejo'] == 'sube las delanteras (L1 · ▲ delanteras)'


def test_sin_alto_del_obstaculo_usa_la_z_del_contacto():
    d = leer(mk.estado_json(contra_escalon('FL'), z_suelo=0.0))
    assert d['motivo'] == 'el morro choca con un canto de 8 cm'


@pytest.mark.parametrize('pieza, motivo, boton', [
    ('RR', 'la cola choca', 'R1'), ('RL', 'la cola choca', 'R1'),
    ('chasis', 'el chasis choca', 'L2 + R2')])
def test_bloqueado_por_la_cola_o_el_chasis(pieza, motivo, boton):
    d = leer(mk.estado_json(contra_escalon(pieza)))
    assert d['pieza'] == pieza and d['motivo'] == motivo
    assert boton in d['consejo']


@pytest.mark.parametrize('bandera, motivo, consejo', [
    ('sin_traccion', 'apoya la panza y las orugas no tocan',
     'baja los brazos (L2 / R2)'),
    ('volcado', 'más de 75° de inclinación',
     'reinicia: /mundo/reiniciar o 2D Pose Estimate en RViz'),
    ('atrapado', 'empujado por los dos lados', 'mueve los brazos o retrocede'),
])
def test_textos_de_cada_estado(bandera, motivo, consejo):
    d = leer(mk.estado_json(estado(**{bandera: True})))
    assert (d['motivo'], d['consejo']) == (motivo, consejo)


def test_el_espejo_manda_sobre_el_estado():
    d = leer(mk.estado_json(contra_escalon(), modo='espejo'))
    assert d['modo'] == 'espejo'
    assert (d['motivo'], d['consejo']) == mk.ESPEJO


def test_el_aviso_solo_sale_con_el_robot_libre():
    aviso = ('mundo no válido: falta formato', 'corrige el YAML')
    assert leer(mk.estado_json(estado(), aviso=aviso))['motivo'] == aviso[0]
    d = leer(mk.estado_json(estado(sin_traccion=True), aviso=aviso))
    assert d['motivo'] == 'apoya la panza y las orugas no tocan'


def test_lo_no_finito_va_como_null():
    d = leer(mk.estado_json(estado(margen=float('nan')), ms=float('inf')))
    assert d['margen'] is None and d['ms'] is None


def test_redondeo_de_las_holguras():
    d = leer(mk.estado_json(estado(holguras=(0.0123456, 0.0, 0.0, 0.1))))
    assert d['holgura'] == [0.0123, 0.0, 0.0, 0.1]


# --- Indicadores ----------------------------------------------------------

@pytest.mark.parametrize('e', [
    estado(), contra_escalon(), estado(volcado=True), estado(poligono=()),
    estado(contactos=(), poligono=())])
def test_contrato_de_claves_de_los_indicadores(e):
    piezas = mk.indicadores(e)
    vistos = set()
    for p in piezas:
        assert tuple(sorted(p)) == tuple(sorted(mk.CLAVES))
        assert p['tipo'] in mk.TIPO_MARKER
        assert (p['ns'], p['id']) not in vistos
        vistos.add((p['ns'], p['id']))
        assert len(p['color']) == 4 and all(0.0 <= c <= 1.0 for c in p['color'])
        assert p['colores'] is None
        assert all(len(q) == 3 and all(math.isfinite(v) for v in q)
                   for q in p['puntos'])
        if p['tipo'] == 'texto':
            assert p['texto'] and len(p['pos']) == 3
        else:
            assert p['puntos'] and p['texto'] is None and p['pos'] is None


def test_choque_solo_si_esta_bloqueado():
    assert 'choque' not in por_ns(mk.indicadores(estado()))
    flecha = por_ns(mk.indicadores(contra_escalon()))['choque']
    assert flecha['tipo'] == 'flecha' and flecha['color'] == mk.AMBAR
    (x0, y0, z0), (x1, y1, z1) = flecha['puntos']
    assert (x0, y0, z0) == (1.0, 0.0, 0.0764)
    assert (x1 - x0, y1 - y0, z1 - z0) == pytest.approx((-0.15, 0.0, 0.0))


def test_rotulo_solo_si_no_esta_libre():
    assert 'estado' not in por_ns(mk.indicadores(estado()))
    rotulo = por_ns(mk.indicadores(contra_escalon('RL')))['estado']
    assert rotulo['texto'] == 'BLOQUEADO (RL)' and rotulo['color'] == mk.AMBAR
    assert rotulo['pos'] == pytest.approx((1.0, 2.0, 0.0764 + 0.30))
    assert rotulo['escala'] == 0.08
    volcado = por_ns(mk.indicadores(estado(volcado=True)))['estado']
    assert volcado['texto'] == 'VOLCADO' and volcado['color'] == mk.ROJO


@pytest.mark.parametrize('margen, color', [
    (0.24, mk.GRIS), (0.1001, mk.GRIS), (0.10, mk.AMBAR), (0.05, mk.AMBAR),
    (0.03, mk.AMBAR), (0.0299, mk.ROJO), (-0.02, mk.ROJO),
    (float('nan'), mk.ROJO)])
def test_color_del_poligono_y_del_cdg_segun_el_margen(margen, color):
    ind = por_ns(mk.indicadores(estado(margen=margen)))
    assert ind['poligono']['color'] == color
    assert ind['cdg']['color'] == color


def test_poligono_cerrado_y_cinco_mm_por_encima():
    e = estado()
    tira = por_ns(mk.indicadores(e))['poligono']
    assert tira['tipo'] == 'tira' and tira['escala'] == 0.012
    assert tira['puntos'][0] == tira['puntos'][-1]
    assert len(tira['puntos']) == len(e.poligono) + 1
    assert all(z == pytest.approx(0.005) for _, _, z in tira['puntos'])


def test_contactos_de_las_bandas_y_de_la_panza():
    e = estado(panza=True, contactos=(
        contacto('FR', 1.3, 1.76), contacto('FL', 1.3, 2.24, holgura=0.004),
        contacto('RR', 0.7, 1.76, tipo='pared'), contacto('chasis', 1.0, 2.0, 0.02)))
    ind = por_ns(mk.indicadores(e))
    assert ind['contactos']['puntos'] == [(1.3, 1.76, 0.0)]
    assert ind['contactos']['color'] == mk.BLANCO
    assert ind['contactos']['escala'] == 0.035
    assert ind['panza']['puntos'] == [(1.0, 2.0, 0.02)]
    assert ind['panza']['color'] == mk.AMBAR
    assert 'panza' not in por_ns(mk.indicadores(estado()))


def test_cdg_y_plomada_hasta_el_contacto_mas_bajo():
    e = estado(contactos=(contacto('FR', z=0.2), contacto('RR', z=0.05)))
    ind = por_ns(mk.indicadores(e))
    assert ind['cdg']['puntos'] == [(1.0, 2.0, 0.08)]
    assert ind['cdg']['escala'] == 0.045
    assert ind['plomada']['tipo'] == 'lineas'
    assert ind['plomada']['puntos'] == [(1.0, 2.0, 0.08), (1.0, 2.0, 0.05)]


def test_dentro_del_choque_va_contra_la_normal():
    c = contacto(x=1.0, y=0.0, normal=(-0.6, 0.0, 0.8))
    assert mk.dentro_del_choque(c) == pytest.approx((1.01, 0.0))
    assert mk.dentro_del_choque(contacto(normal=(0.0, 0.0, 1.0))) is None


# --- Cuaternion de /mundo/verdad -------------------------------------------

def rotar(q, v):
    x, y, z, w = q
    u = (x, y, z)

    def cruz(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])
    t = tuple(2.0 * c for c in cruz(u, v))
    c = cruz(u, t)
    return tuple(v[i] + w * t[i] + c[i] for i in range(3))


def test_cuaternion_zyx_como_las_juntas():
    assert mk.cuaternion_zyx(0.7, 0.0, 0.0) == pytest.approx(
        (0.0, 0.0, math.sin(0.35), math.cos(0.35)))
    q = mk.cuaternion_zyx(0.3, -0.2, 0.1)
    assert sum(c * c for c in q) == pytest.approx(1.0)
    # REP-103: cabeceo + = morro abajo, balanceo + = lado izquierdo arriba
    assert rotar(mk.cuaternion_zyx(0.0, 0.2, 0.0), (1.0, 0.0, 0.0))[2] < 0.0
    assert rotar(mk.cuaternion_zyx(0.0, 0.0, 0.2), (0.0, 1.0, 0.0))[2] > 0.0
    # El rumbo que saca la GUI de /mundo/verdad es el de la TF
    x, y, z, w = mk.cuaternion_zyx(2.5, 0.3, -0.2)
    assert math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) == \
        pytest.approx(2.5)


# --- Del dict al Marker ---------------------------------------------------

def test_tipos_como_los_de_visualization_msgs():
    msg = pytest.importorskip('visualization_msgs.msg')
    M = msg.Marker
    assert mk.TIPO_MARKER == {
        'triangulos': M.TRIANGLE_LIST, 'lineas': M.LINE_LIST,
        'tira': M.LINE_STRIP, 'esferas': M.SPHERE_LIST, 'flecha': M.ARROW,
        'texto': M.TEXT_VIEW_FACING}


def test_a_markers_del_mundo_y_de_los_indicadores():
    msg = pytest.importorskip('visualization_msgs.msg')
    from builtin_interfaces.msg import Time
    M = msg.Marker
    terreno = {'ns': 'terreno', 'id': 0, 'tipo': 'triangulos',
               'puntos': [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
               'colores': [(0.4, 0.4, 0.4, 1)] * 3, 'color': (1, 1, 1, 1),
               'escala': 1.0, 'texto': None, 'pos': None}
    etiqueta = {'ns': 'etiquetas', 'id': 2, 'tipo': 'texto', 'puntos': [],
                'colores': None, 'color': (0.76, 0.76, 0.72, 1), 'escala': 0.08,
                'texto': 'Carril 1', 'pos': (1.6, 0.6, 0.5)}
    sello = Time(sec=12, nanosec=34)
    arr = mk.a_markers([terreno, etiqueta] + mk.indicadores(contra_escalon()),
                       'odom', sello, vida_s=0.5)
    primero = arr.markers[0]
    assert primero.action == M.DELETEALL
    resto = arr.markers[1:]
    assert all(m.action == M.ADD and m.header.frame_id == 'odom'
               and m.header.stamp == sello for m in resto)
    assert all(m.lifetime.sec == 0 and m.lifetime.nanosec == 500000000
               for m in resto)
    t, e = resto[0], resto[1]
    assert t.type == M.TRIANGLE_LIST and len(t.points) == 3
    assert (t.scale.x, t.scale.y, t.scale.z) == (1.0, 1.0, 1.0)
    assert len(t.colors) == 3 and t.colors[0].r == pytest.approx(0.4)
    assert t.pose.orientation.w == 1.0 and t.pose.position.x == 0.0
    assert e.type == M.TEXT_VIEW_FACING and e.text == 'Carril 1'
    assert (e.pose.position.x, e.pose.position.z) == (1.6, 0.5)
    assert e.scale.z == 0.08 and e.id == 2 and not e.points
    ind = {m.ns: m for m in resto[2:]}
    assert ind['contactos'].type == M.SPHERE_LIST
    assert ind['contactos'].scale.x == ind['contactos'].scale.z == 0.035
    assert ind['poligono'].type == M.LINE_STRIP
    assert ind['poligono'].scale.x == 0.012 and ind['poligono'].scale.y == 0.0
    assert ind['plomada'].type == M.LINE_LIST
    flecha = ind['choque']
    assert flecha.type == M.ARROW and len(flecha.points) == 2
    assert (flecha.scale.x, flecha.scale.y, flecha.scale.z) == \
        pytest.approx((0.02, 0.04, 0.05))
    assert ind['estado'].color.r == pytest.approx(mk.AMBAR[0])
    assert not ind['estado'].colors


def test_a_markers_sin_piezas_es_solo_el_deleteall():
    msg = pytest.importorskip('visualization_msgs.msg')
    from builtin_interfaces.msg import Time
    arr = mk.a_markers([], 'odom', Time())
    assert [m.action for m in arr.markers] == [msg.Marker.DELETEALL]
    assert arr.markers[0].lifetime.sec == 0


@pytest.mark.parametrize('cambio', [
    {'tipo': 'cubo'}, {'puntos': [(0, 0, 0)] * 4},
    {'colores': [(1, 1, 1, 1)]}])
def test_a_markers_rechaza_lo_que_no_es_del_contrato(cambio):
    pytest.importorskip('visualization_msgs.msg')
    from builtin_interfaces.msg import Time
    p = {'ns': 'terreno', 'id': 0, 'tipo': 'triangulos',
         'puntos': [(0, 0, 0), (1, 0, 0), (0, 1, 0)], 'colores': None,
         'color': (1, 1, 1, 1), 'escala': 1.0, 'texto': None, 'pos': None}
    p.update(cambio)
    with pytest.raises(ValueError, match='terreno/0'):
        mk.a_markers([p], 'odom', Time())
