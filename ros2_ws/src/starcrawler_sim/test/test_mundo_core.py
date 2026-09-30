"""Tests de mundo_core y mundo_check. Puros: no necesitan ROS, salvo la
conversion a Marker, que se salta sin visualization_msgs."""
import dataclasses
import glob
import math
import os
import random
import re
from collections import Counter

import pytest

from starcrawler_sim import mundo_check
from starcrawler_sim.mundo_core import (
    ErrorMundo,
    Mundo,
    Solido,
    cargar,
    cargar_fichero,
    resolver_ruta,
)

MUNDOS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       os.pardir, 'mundos'))
FICHEROS = sorted(glob.glob(os.path.join(MUNDOS, '*.yaml')))
EJEMPLOS = ('llano', 'escalon', 'rampa', 'escalera', 'practica_rrl')
T15 = math.tan(math.radians(15))
CLAVES_VISUAL = {'ns', 'id', 'tipo', 'puntos', 'colores', 'color', 'escala',
                 'texto', 'pos'}


def mundo(nombre):
    return cargar_fichero(os.path.join(MUNDOS, nombre + '.yaml'))


def perfil_aprox(obtenido, esperado, tol=1e-9):
    assert len(obtenido) == len(esperado), obtenido
    for (s, z), (se, ze) in zip(obtenido, esperado):
        assert s == pytest.approx(se, abs=tol) and z == pytest.approx(ze, abs=tol), obtenido


def en_perfil(perfil, s):
    """z de la polilinea en s, o None si s cae en un vertice."""
    for (sa, za), (sb, zb) in zip(perfil, perfil[1:]):
        if sa < s < sb:
            return za + (zb - za) * (s - sa) / (sb - sa)
    return None


# --- API -----------------------------------------------------------------

def test_solido_es_el_de_la_especificacion():
    campos = [f.name for f in dataclasses.fields(Solido)]
    assert campos == ['cx', 'cy', 'rumbo', 'largo', 'ancho', 'a', 'bx', 'by',
                      'clase', 'color', 'origen']
    so = Solido(0.0, 0.0, 0.0, 1.0, 1.0, 0.1, 0.0, 0.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        so.cx = 1.0


def test_error_mundo_es_value_error():
    assert issubclass(ErrorMundo, ValueError)


def test_mundo_vacio_es_llano():
    m = Mundo()
    assert m.altura(3.0, -2.0) == 0.0
    assert m.perfil(1.0, 1.0, 0.0, 1.0, -1.0, 1.0) == [(-1.0, 0.0), (1.0, 0.0)]
    assert m.visual() == []


# --- Arenas de ejemplo -----------------------------------------------------

def test_cargan_todas_las_arenas():
    nombres = {os.path.splitext(os.path.basename(f))[0] for f in FICHEROS}
    assert set(EJEMPLOS) <= nombres
    for ruta in FICHEROS:
        m = cargar_fichero(ruta)
        assert m.nombre == os.path.splitext(os.path.basename(ruta))[0]
        assert m.resumen().startswith('mundo ' + m.nombre)


def test_llano():
    m = mundo('llano')
    assert m.inicio == (0.0, 0.0, 0.0)
    assert m.solidos == () and m.visual() == []
    assert m.perfil(0.0, 0.0, 1.0, 0.0, -1.0, 1.0) == [(-1.0, 0.0), (1.0, 0.0)]


def test_escalon_altura_con_borde_cerrado():
    m = mundo('escalon')
    assert m.altura(1.5, 0.0) == pytest.approx(0.20, abs=1e-12)
    assert m.altura(0.99, 0.0) == 0.0
    assert m.altura(1.0, 0.0) == pytest.approx(0.20, abs=1e-12)
    assert m.altura(2.0, 0.6) == pytest.approx(0.20, abs=1e-12)
    assert m.altura(1.5, 0.61) == 0.0


def test_escalon_perfil_exacto():
    m = mundo('escalon')
    perfil_aprox(m.perfil(0.0, 0.0, 1.0, 0.0, 0.0, 2.5),
                 [(0, 0), (1.0, 0), (1.0, 0.2), (2.0, 0.2), (2.0, 0), (2.5, 0)])
    # Hacia atras: el salto va primero con el valor de la izquierda
    perfil_aprox(m.perfil(2.5, 0.0, -1.0, 0.0, 0.0, 1.0),
                 [(0, 0), (0.5, 0), (0.5, 0.2), (1.0, 0.2)])


def test_rampa():
    m = mundo('rampa')
    largo = 0.20 / T15
    assert largo == pytest.approx(0.746, abs=1e-3)
    assert m.altura(1.0 + largo / 2, 0.0) == pytest.approx(0.10, abs=1e-9)
    assert m.altura(1.0 + largo + 0.4, 0.0) == pytest.approx(0.20, abs=1e-9)
    fin = 1.0 + largo + 0.8 + 0.20 / math.tan(math.radians(30))
    assert fin == pytest.approx(2.892, abs=1e-3)
    perfil_aprox(m.perfil(0.0, 0.0, 1.0, 0.0, 0.0, 3.5),
                 [(0, 0), (1.0, 0), (1.0 + largo, 0.2), (1.8 + largo, 0.2),
                  (fin, 0), (3.5, 0)])


def test_escalera_peldanos_y_rellano():
    m = mundo('escalera')
    for k in (1, 2, 3):
        x = 1.2 + (k - 1) * 0.25 + 0.125
        assert m.altura(x, 0.0) == pytest.approx(0.20 * k, abs=1e-9)
    assert m.altura(1.96, 0.0) == pytest.approx(0.80, abs=1e-9)
    assert m.altura(2.94, 0.3) == pytest.approx(0.80, abs=1e-9)
    for k, x in ((3, 3.05), (2, 3.30), (1, 3.55)):
        assert m.altura(x, 0.0) == pytest.approx(0.20 * k, abs=1e-9)
    assert m.altura(3.71, 0.0) == 0.0
    p = m.perfil(0.0, 0.0, 1.0, 0.0, 0.0, 4.0)
    saltos = [round(zb - za, 9) for (sa, za), (sb, zb) in zip(p, p[1:]) if sa == sb]
    assert saltos == [0.2] * 4 + [-0.2] * 4


# --- practica_rrl ----------------------------------------------------------

def test_practica_inicio_es_el_origen_de_odom():
    m = mundo('practica_rrl')
    assert m.inicio == (0.80, 0.60, 0.0)
    salida = next(z for z in m.zonas if z.nombre == 'salida')
    assert salida.centro == pytest.approx((0.0, 0.0), abs=1e-12)


def test_practica_bordillo_del_carril_1():
    m = mundo('practica_rrl')
    for y in (-0.55, -0.3, 0.0, 0.3, 0.55):
        assert m.altura(1.25, y) == pytest.approx(0.10, abs=1e-12)
    assert m.altura(1.20, 0.0) == pytest.approx(0.10, abs=1e-12)
    assert m.altura(1.30, 0.0) == pytest.approx(0.10, abs=1e-12)
    assert m.altura(1.199, 0.0) == 0.0 and m.altura(1.301, 0.0) == 0.0
    perfil_aprox(m.perfil(1.0, 0.0, 1.0, 0.0, 0.0, 0.5),
                 [(0, 0), (0.2, 0), (0.2, 0.1), (0.3, 0.1), (0.3, 0), (0.5, 0)])


def test_practica_rampas_con_sigue():
    m = mundo('practica_rrl')
    la = 0.16 / T15
    # En el carril 1 (rumbo 0), s del carril = x de odom - 0,8
    perfil_aprox(m.perfil(0.8, 0.0, 1.0, 0.0, 1.0, 3.0),
                 [(1.0, 0), (1.1, 0), (1.1 + la, 0.16), (1.7 + la, 0.16),
                  (1.7 + 2 * la, 0), (3.0, 0)])


def test_practica_carril_con_rumbo_180_y_sigue():
    m = mundo('practica_rrl')
    c2 = next(c for c in m.carriles if c.nombre == 'carril_2')
    assert c2.origen == pytest.approx((5.6, 1.2), abs=1e-12)
    assert abs(c2.rumbo) == pytest.approx(math.pi, abs=1e-12)
    perfil_aprox(m.perfil(5.6, 1.2, -1.0, 0.0, 0.0, 4.8),
                 [(0, 0), (0.4, 0), (0.4, 0.1), (1.4, 0.1), (1.4, 0), (2.0, 0),
                  (2.0, 0.2), (3.0, 0.2), (3.0, 0), (3.4, 0), (3.4, 0.2), (4.0, 0.2),
                  (4.0, 0.4), (4.6, 0.4), (4.6, 0), (4.8, 0)])
    assert (c2.s_min, c2.s_max, c2.z_max) == pytest.approx((0.4, 4.6, 0.4), abs=1e-9)


def test_practica_paredes():
    m = mundo('practica_rrl')
    assert sum(so.clase == 'pared' for so in m.solidos) == 8
    # Tabique en y = 1,2 del fichero (0,6 de odom), de x = 0 a 6,4
    assert m.altura(0.0, 0.6) == pytest.approx(0.8)
    assert m.altura(5.605, 0.6) == pytest.approx(0.8)     # la esquina cierra
    assert m.altura(5.62, 0.6) == 0.0
    assert m.altura(0.0, 0.6, paredes=False) == 0.0


# --- Campos ----------------------------------------------------------------

def centro_carril_4(s0, i, j, celda=0.10, columnas=12):
    """Centro de la celda (i, j) de un campo del carril 4 (rumbo 180), en odom."""
    s = s0 + (i + 0.5) * celda
    t = columnas * celda / 2 - (j + 0.5) * celda
    return 6.4 - s - 0.8, 4.2 - t - 0.6


def test_campos_numero_de_postes():
    m = mundo('practica_rrl')
    postes = Counter(so.origen for so in m.solidos)
    assert postes['carril_4, elemento 1 (campo_escalones)'] == 144
    assert postes['carril_4, elemento 2 (campo_escalones)'] == 144
    assert postes['carril_1, elemento 5 (campo_rampas)'] == 4
    # Con a = 0 no hay poste
    m = cargar('formato: 1\nelementos:\n'
               '  - {tipo: campo_escalones, pos: [0, 0], alturas: [[1, 0], [0, 2], [3, 0]]}\n')
    assert len(m.solidos) == 3
    assert m.altura(0.05, 0.05) == pytest.approx(0.10)       # fila 0, columna 0
    assert m.altura(0.05, -0.05) == 0.0                      # fila 0, columna 1
    assert m.altura(0.15, -0.05) == pytest.approx(0.20)
    assert m.altura(0.25, 0.05) == pytest.approx(0.30)


def test_campo_plano_cruz():
    m = mundo('practica_rrl')
    alturas = {}
    for i in range(12):
        for j in range(12):
            alturas[i, j] = round(m.altura(*centro_carril_4(0.3, i, j)), 9)
    assert Counter(alturas.values()) == {0.05: 96, 0.10: 44, 0.15: 4}
    for i, j in ((2, 2), (2, 9), (9, 2), (9, 9)):
        assert alturas[i, j] == 0.15
    for k in range(12):
        assert alturas[5, k] == alturas[6, k] == alturas[k, 5] == alturas[k, 6] == 0.10
    assert alturas[0, 0] == alturas[11, 11] == 0.05


def test_campo_colina_explicita():
    m = mundo('practica_rrl')
    assert m.altura(*centro_carril_4(1.9, 0, 0)) == pytest.approx(0.40)
    assert m.altura(*centro_carril_4(1.9, 0, 11)) == pytest.approx(0.10)
    assert m.altura(*centro_carril_4(1.9, 11, 11)) == pytest.approx(0.40)
    assert m.altura(*centro_carril_4(1.9, 3, 5)) == pytest.approx(0.30)


def test_rampas_cruzadas_a_media_altura_en_el_centro():
    m = mundo('practica_rrl')
    media = 0.6 * T15 / 2
    # Celda (0, 0), A: sube hacia +s (+x de odom); centro en s = 3,6, t = 0,3
    assert m.altura(4.4, 0.3) == pytest.approx(media, abs=1e-9)
    assert m.altura(4.6, 0.3) - m.altura(4.2, 0.3) == pytest.approx(0.4 * T15, abs=1e-9)
    assert m.altura(4.4, 0.5) == pytest.approx(media, abs=1e-9)
    # Celda (0, 1), I: sube hacia +t
    assert m.altura(4.4, -0.3) == pytest.approx(media, abs=1e-9)
    assert m.altura(4.4, -0.1) - m.altura(4.4, -0.5) == pytest.approx(0.4 * T15, abs=1e-9)
    assert m.altura(4.2, -0.3) == pytest.approx(m.altura(4.6, -0.3), abs=1e-9)


def test_campo_rampas_orientaciones():
    m = cargar('formato: 1\nelementos:\n'
               '  - {tipo: campo_rampas, pos: [0, 0], celda: 1.0, pendiente: 10,\n'
               '     orientaciones: [[A, T, I, D]]}\n')
    t10 = math.tan(math.radians(10))
    centros = [(0.5, 1.5), (0.5, 0.5), (0.5, -0.5), (0.5, -1.5)]
    subida = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for (x, y), (dx, dy) in zip(centros, subida):
        assert m.altura(x, y) == pytest.approx(t10 / 2, abs=1e-9)
        assert (m.altura(x + 0.3 * dx, y + 0.3 * dy) - m.altura(x - 0.3 * dx, y - 0.3 * dy)
                == pytest.approx(0.6 * t10, abs=1e-9))


# --- Marcos: todo sale en odom ---------------------------------------------

def test_inicio_girado_y_carril_girado():
    m = cargar(
        'formato: 1\n'
        'inicio: {x: 1.0, y: 2.0, rumbo: 90}\n'
        'zonas: [{nombre: z, de: [1, 2], a: [3, 3]}]\n'
        'elementos:\n'
        '  - {tipo: caja, pos: [2.0, 2.0], largo: 1.0, ancho: 0.5, alto: 0.3}\n'
        'carriles:\n'
        '  - {nombre: c, origen: [1.0, 2.0], rumbo: 90, ancho: 1.0, elementos: [\n'
        '      {tipo: rampa, pos: [1.0, 0], largo: 1.0, alto: 0.2}]}\n')
    # La caja del fichero (x de 2 a 3, y de 1,75 a 2,25), girada -90 grados
    assert m.altura(0.0, -1.5) == pytest.approx(0.3)
    assert m.altura(0.2, -1.1) == pytest.approx(0.3)
    assert m.altura(0.3, -1.5) == 0.0 and m.altura(0.0, -0.95) == 0.0
    caja = m.solidos[0]
    assert caja.rumbo == pytest.approx(-math.pi / 2)
    assert (caja.cx, caja.cy) == pytest.approx((0.0, -1.5))
    # El carril con rumbo 90 en el fichero va por +x de odom
    c = m.carriles[0]
    assert c.origen == pytest.approx((0.0, 0.0)) and c.rumbo == pytest.approx(0.0)
    rampa = m.solidos[1]
    assert (rampa.bx, rampa.by) == pytest.approx((0.2, 0.0), abs=1e-12)
    assert m.altura(1.5, 0.0) == pytest.approx(0.1)
    assert m.zonas[0].centro == pytest.approx((0.5, -1.0))


def test_rumbo_de_una_caja_y_viga_diagonal():
    m = cargar('formato: 1\n'
               'carriles:\n'
               '  - {nombre: c, origen: [0, 0], rumbo: 30, ancho: 1.0, elementos: [\n'
               '      {tipo: caja, pos: [1.0, 0], rumbo: 60, largo: 0.4, alto: 0.1},\n'
               '      {tipo: viga, de: [3, 1], a: [4, 2], seccion: 0.2}]}\n')
    caja, viga = m.solidos
    assert caja.rumbo == pytest.approx(math.pi / 2)
    assert (caja.cx, caja.cy) == pytest.approx((math.cos(math.pi / 6), 0.5 + 0.2))
    assert viga.rumbo == pytest.approx(math.radians(75))
    assert viga.largo == pytest.approx(math.sqrt(2)) and viga.ancho == 0.2
    assert viga.a == pytest.approx(0.2)                 # alto = seccion


# --- Propiedad de perfil() --------------------------------------------------

def test_perfil_coincide_con_altura_en_practica_rrl():
    m = mundo('practica_rrl')
    rnd = random.Random(20260930)
    for n in range(500):
        if n % 4 == 1:
            x0, y0 = rnd.uniform(2.5, 5.3), rnd.uniform(3.0, 4.2)    # campos
        else:
            x0, y0 = rnd.uniform(-1.0, 7.4), rnd.uniform(-0.8, 5.6)
        if n % 4 == 0:
            a = rnd.randrange(4) * math.pi / 2          # por los carriles
        else:
            a = rnd.uniform(-math.pi, math.pi)
        ux, uy = math.cos(a), math.sin(a)
        s0 = rnd.uniform(-1.5, 0.0)
        s1 = s0 + rnd.uniform(0.05, 3.0)
        p = m.perfil(x0, y0, ux, uy, s0, s1)
        assert p[0][0] == s0 and p[-1][0] == s1
        assert all(sb >= sa for (sa, _), (sb, _) in zip(p, p[1:]))
        # Un salto son dos vertices, nunca tres con la misma s
        assert all(not (q0[0] == q1[0] == q2[0]) for q0, q1, q2 in zip(p, p[1:], p[2:]))
        assert all(z >= 0.0 and math.isfinite(z) for _, z in p)
        saltos = [sa for (sa, _), (sb, _) in zip(p, p[1:]) if sa == sb]
        for _ in range(20):
            s = rnd.uniform(s0, s1)
            if any(abs(s - sj) < 1e-6 for sj in saltos):
                continue
            z = en_perfil(p, s)
            if z is not None:
                assert z == pytest.approx(m.altura(x0 + s * ux, y0 + s * uy), abs=1e-9)
        # En un salto, el primer vertice es el de la izquierda
        for k in range(len(p) - 1):
            (sa, za), (sb, zb) = p[k], p[k + 1]
            if sa != sb:
                continue
            antes = p[k - 1][0] if k > 0 else -math.inf
            despues = p[k + 2][0] if k + 2 < len(p) else math.inf
            if sa - antes > 2e-6 and sa > s0:
                s = sa - 1e-6
                assert za == pytest.approx(m.altura(x0 + s * ux, y0 + s * uy), abs=1e-5)
            if despues - sb > 2e-6 and sb < s1:
                s = sb + 1e-6
                assert zb == pytest.approx(m.altura(x0 + s * ux, y0 + s * uy), abs=1e-5)


def test_perfil_s1_menor_que_s0():
    with pytest.raises(ValueError):
        mundo('escalon').perfil(0.0, 0.0, 1.0, 0.0, 1.0, 0.0)


# --- Errores ---------------------------------------------------------------

def error(texto, nombre_fichero=''):
    with pytest.raises(ErrorMundo) as e:
        cargar(texto, nombre_fichero)
    return str(e.value)


def suelto(elemento):
    return 'formato: 1\nelementos:\n  - %s\n' % elemento


def test_error_rampa_con_tres_cotas():
    m = error(suelto('{tipo: rampa, pos: [0, 0], ancho: 1, largo: 1, alto: 0.2, pendiente: 10}'))
    assert m.startswith('elementos, elemento 1 (rampa): ')
    assert 'exactamente dos de largo/alto/pendiente' in m


def test_error_tipo_desconocido():
    m = error(suelto('{tipo: piramide, pos: [0, 0], ancho: 1}'))
    assert m.startswith('elementos, elemento 1 (piramide): tipo desconocido')


def test_error_tubo_no_soportado():
    m = error(suelto('{tipo: tubo, pos: [0, 0], ancho: 1}'))
    assert m == 'elementos, elemento 1 (tubo): tipo tubo no soportado en formato 1'
    m = error(suelto('{tipo: caja, pos: [0, 0], ancho: 1, largo: 1, alto: 0.1, encima: 1}'))
    assert "'encima' no soportado en formato 1" in m


def test_error_clave_desconocida():
    m = error(suelto('{tipo: caja, pos: [0, 0], ancho: 1, largo: 1, alto: 0.1, altura: 3}'))
    assert m == "elementos, elemento 1 (caja): clave desconocida 'altura'"
    assert error('formato: 1\nobstaculos: []\n') == \
        "nivel superior: clave desconocida 'obstaculos'"
    m = error(suelto('{tipo: campo_escalones, pos: [0, 0], rumbo: 90, alturas: [[1]]}'))
    assert "'rumbo' no se admite en campo_escalones" in m


def test_error_matriz_no_rectangular():
    m = error(suelto('{tipo: campo_escalones, pos: [0, 0], alturas: [[1, 2], [1]]}'))
    assert m.startswith('elementos, elemento 1 (campo_escalones): ')
    assert "'alturas' no es rectangular" in m


def test_error_dice_donde_esta():
    texto = ('formato: 1\ncarriles:\n  - origen: [0, 0]\n    ancho: 1.2\n    elementos:\n'
             '      - {tipo: caja, pos: [0, 0], largo: 1, alto: 0.1}\n'
             '      - {tipo: viga, de: [1, 0]}\n')
    assert error(texto) == "carril_1, elemento 2 (viga): falta 'a'"
    assert error(texto, 'roto.yaml') == "roto.yaml: carril_1, elemento 2 (viga): falta 'a'"


@pytest.mark.parametrize('texto, mensaje', [
    ('nombre: x\n', "falta 'formato: 1'"),
    ('formato: 2\n', 'formato 2 no soportado'),
    ('formato: 1\nformato: 1\n', "clave repetida 'formato'"),
    ('formato: 1\nelementos: [\n', 'YAML mal formado'),
    (suelto('{tipo: caja, pos: [sigue, 0], ancho: 1, largo: 1, alto: 0.1}'),
     "'sigue' sin elemento anterior"),
    (suelto('{tipo: caja, pos: [0, 0], ancho: 1, largo: .nan, alto: 0.1}'),
     "'largo' debe ser un numero finito"),
    (suelto('{tipo: caja, pos: [0, 0], ancho: 1, largo: 0, alto: 0.1}'),
     "'largo' debe ser > 0"),
    (suelto('{tipo: caja, pos: [0, 0], largo: 1, alto: 0.1}'), "falta 'ancho'"),
    (suelto('{tipo: rampa, pos: [0, 0], ancho: 1, z: -0.1, largo: 1, alto: 0.2}'),
     "'z' debe ser >= 0"),
    (suelto('{tipo: rampa, pos: [0, 0], ancho: 1, z: 0.1, largo: 1, alto: -0.2}'),
     'por debajo del suelo'),
    (suelto('{tipo: campo_escalones, pos: [0, 0], alturas: [[1, -1]]}'),
     "'alturas' lleva enteros >= 0"),
    (suelto('{tipo: campo_rampas, pos: [0, 0], orientaciones: [[A, X]]}'),
     'orientacion'),
    (suelto('{tipo: caja, pos: [0, 0], ancho: 1, largo: 1, alto: 0.1, color: [1, 2, 0]}'),
     "'color' va de 0 a 1"),
])
def test_errores_de_validacion(texto, mensaje):
    assert mensaje in error(texto)


def test_nombre_del_fichero_si_no_lo_lleva():
    assert cargar('formato: 1\n', '/tmp/mi_pista.yaml').nombre == 'mi_pista'


# --- visual() --------------------------------------------------------------

def triangulos(pieza):
    pts = pieza['puntos']
    return [pts[k:k + 3] for k in range(0, len(pts), 3)]


def normal(p0, p1, p2):
    ax, ay, az = (p1[i] - p0[i] for i in range(3))
    bx, by, bz = (p2[i] - p0[i] for i in range(3))
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx


def comprobar_hacia_fuera(so):
    """Cada triangulo del solido mira hacia fuera: n . (cara - interior) > 0."""
    vis = Mundo('uno', [so]).visual()
    ns = 'paredes' if so.clase == 'pared' else 'terreno'
    pieza = next(d for d in vis if d['ns'] == ns)
    interior = (so.cx, so.cy, so.techo(so.cx, so.cy) / 2)
    for p0, p1, p2 in triangulos(pieza):
        n = normal(p0, p1, p2)
        cara = [(p0[i] + p1[i] + p2[i]) / 3 for i in range(3)]
        assert sum(n[i] * (cara[i] - interior[i]) for i in range(3)) > 0, (so.origen, p0, p1, p2)


def test_malla_normales_hacia_fuera():
    raro = cargar('formato: 1\ncarriles:\n'
                  '  - {nombre: c, origen: [0, 0], rumbo: 30, ancho: 1.0, elementos: [\n'
                  '      {tipo: caja, pos: [1.0, 0], rumbo: 60, largo: 0.4, alto: 0.1},\n'
                  '      {tipo: rampa, pos: [2.0, 0], rumbo: -20, largo: 0.5, alto: 0.2},\n'
                  '      {tipo: campo_rampas, pos: [3, 0], celda: 0.5,\n'
                  '       orientaciones: [[A, T], [I, D]]}]}\n')
    solidos = raro.solidos + mundo('rampa').solidos + mundo('practica_rrl').solidos
    assert {so.clase for so in solidos} == {'terreno', 'pared'}
    for so in solidos:
        comprobar_hacia_fuera(so)


def test_malla_contrato_colores_e_ids():
    for nombre in EJEMPLOS:
        vis = mundo(nombre).visual()
        claves = [(d['ns'], d['id']) for d in vis]
        assert len(claves) == len(set(claves)), nombre
        for d in vis:
            assert set(d) == CLAVES_VISUAL
            assert d['tipo'] in ('triangulos', 'lineas', 'tira', 'texto')
            numeros = [v for q in d['puntos'] for v in q] + list(d['color']) + [d['escala']]
            if d['pos'] is not None:
                numeros += list(d['pos'])
            if d['colores'] is not None:
                assert len(d['colores']) == len(d['puntos'])
                assert all(len(c) == 4 for c in d['colores'])
                numeros += [v for c in d['colores'] for v in c]
                assert all(0.0 <= v <= 1.0 for c in d['colores'] for v in c)
            assert all(isinstance(v, float) and math.isfinite(v) for v in numeros), d['ns']
            assert len(d['color']) == 4 and all(0.0 <= v <= 1.0 for v in d['color'])
            assert all(len(q) == 3 for q in d['puntos'])
            if d['tipo'] == 'triangulos':
                assert len(d['puntos']) % 3 == 0 and d['puntos']
            if d['tipo'] == 'lineas':
                assert len(d['puntos']) % 2 == 0 and d['puntos']
            if d['tipo'] == 'texto':
                assert d['texto'] and d['pos'] is not None and d['puntos'] == []


def test_malla_del_escalon():
    vis = {d['ns']: d for d in mundo('escalon').visual()}
    assert set(vis) == {'terreno', 'aristas'}
    t = vis['terreno']
    assert len(t['puntos']) == 30                       # tapa y cuatro caras
    assert t['color'][3] == 1.0
    suelo = (0x6b / 255, 0x67 / 255, 0x5f / 255, 1.0)
    alto = (0xcd / 255, 0xc6 / 255, 0xb6 / 255)
    gris_20 = tuple(b + 0.2 * (a - b) for a, b in zip(alto, suelo)) + (1.0,)
    for p, c in zip(t['puntos'], t['colores']):
        assert c == pytest.approx(suelo if p[2] == 0.0 else gris_20)
    a = vis['aristas']
    assert a['tipo'] == 'lineas' and len(a['puntos']) == 16
    assert a['color'] == pytest.approx((0.11, 0.11, 0.10, 0.7)) and a['escala'] == 0.004


def test_malla_color_propio_y_sin_laterales_por_debajo_de_1_mm():
    m = cargar('formato: 1\nelementos:\n'
               '  - {tipo: caja, pos: [0, 0], ancho: 1, largo: 1, alto: 0.2, color: [1, 0, 0]}\n'
               '  - {tipo: caja, pos: [2, 0], ancho: 1, largo: 1, alto: 0.0005}\n')
    vis = {d['ns']: d for d in m.visual()}
    t = vis['terreno']
    assert len(t['puntos']) == 30 + 6
    assert set(t['colores'][:30]) == {(1.0, 0.0, 0.0, 1.0)}
    assert len(vis['aristas']['puntos']) == 16             # solo las de la caja alta


def test_malla_paredes_zonas_y_etiquetas():
    m = mundo('practica_rrl')
    vis = m.visual()
    por_ns = Counter(d['ns'] for d in vis)
    assert por_ns == {'terreno': 1, 'paredes': 1, 'aristas': 1, 'zonas': 5, 'etiquetas': 10}
    paredes = next(d for d in vis if d['ns'] == 'paredes')
    assert paredes['color'] == pytest.approx((0.478, 0.510, 0.549, 0.28))
    assert paredes['colores'] is None
    zonas = [d for d in vis if d['ns'] == 'zonas']
    assert sorted(d['id'] for d in zonas) == list(range(5))
    for d in zonas:
        assert d['tipo'] == 'tira' and len(d['puntos']) == 5
        assert d['puntos'][0] == d['puntos'][-1]
        assert all(q[2] == 0.002 for q in d['puntos'])
        assert d['color'] == (1.0, 1.0, 1.0, 0.5) and d['escala'] == 0.01
    etiquetas = {d['texto']: d for d in vis if d['ns'] == 'etiquetas'}
    assert len(etiquetas) == 10
    for d in etiquetas.values():
        assert d['escala'] == 0.08 and d['color'] == pytest.approx((0.76, 0.76, 0.72, 1.0))
    c2 = etiquetas['2. Escalones de 10 y 20 cm, valla 20 + 20']
    assert c2['pos'] == pytest.approx((5.6, 1.2, 0.4 + 0.4))
    assert etiquetas['salida']['pos'] == pytest.approx((0.0, 0.0, 0.3))
    assert sum(len(d['puntos']) for d in vis) < 60000
    assert m.avisos == []


def test_aviso_de_mas_de_60000_vertices():
    m = cargar('formato: 1\nelementos:\n'
               '  - {tipo: campo_escalones, pos: [0, 0], patron: colina_diagonal,\n'
               '     filas: 37, columnas: 37}\n')
    assert len(m.solidos) == 37 * 37
    assert sum(len(d['puntos']) for d in m.visual()) > 60000
    assert any('vertices' in a for a in m.avisos)
    assert 'aviso: la malla tiene' in m.resumen()


def test_contrato_con_marcadores_a_markers():
    pytest.importorskip('visualization_msgs.msg')
    from builtin_interfaces.msg import Time

    from starcrawler_sim import marcadores
    for nombre in EJEMPLOS:
        vis = mundo(nombre).visual()
        arr = marcadores.a_markers(vis, 'odom', Time())
        assert len(arr.markers) == len(vis) + 1


# --- resumen() y resolver_ruta() --------------------------------------------

def test_resumen_por_carril():
    r = mundo('practica_rrl').resumen()
    assert re.fullmatch(r'mundo practica_rrl: \d+ solidos \(8 de pared\), \d+ triangulos, '
                        r'altura maxima 0\.600 m', r.splitlines()[0])
    assert '  carril_2: s de 0.400 a 4.600 m, altura maxima 0.400 m (4 solidos)' in r


def test_resolver_ruta():
    assert resolver_ruta('otro.yaml') == 'otro.yaml'
    assert resolver_ruta('mundos/x') == 'mundos/x'
    ruta = resolver_ruta('escalon')
    assert os.path.isfile(ruta) and os.path.basename(ruta) == 'escalon.yaml'
    with pytest.raises(ErrorMundo, match="no hay mundo 'no_existe_zzz'"):
        resolver_ruta('no_existe_zzz')


def test_cargar_fichero_que_no_existe(tmp_path):
    with pytest.raises(ErrorMundo, match='no se puede leer'):
        cargar_fichero(str(tmp_path / 'nada.yaml'))


# --- mundo_check -----------------------------------------------------------

def test_mundo_check_con_las_de_ejemplo(capsys):
    assert mundo_check.main(FICHEROS) == 0
    salida = capsys.readouterr().out
    assert 'mundo practica_rrl: ' in salida and 'ERROR' not in salida


def test_mundo_check_con_un_yaml_roto(tmp_path, capsys):
    roto = tmp_path / 'roto.yaml'
    roto.write_text(suelto('{tipo: viga, de: [0, 0]}'), encoding='utf-8')
    assert mundo_check.main([str(roto)]) == 1
    assert "ERROR: %s: elementos, elemento 1 (viga): falta 'a'" % roto in capsys.readouterr().out
    assert mundo_check.main([FICHEROS[0], str(roto)]) == 1
    assert mundo_check.main([str(tmp_path / 'no_esta.yaml')]) == 1


def avisos(texto):
    return mundo_check.avisos(cargar(texto))


def test_mundo_check_avisa_de_saltos():
    a = avisos(suelto('{tipo: caja, pos: [1, 0], ancho: 1, largo: 1, alto: 0.45}'))
    assert len(a) == 1 and 'salto vertical de 0.450 m' in a[0]
    assert avisos(suelto('{tipo: caja, pos: [1, 0], ancho: 1, largo: 1, alto: 0.40}')) == []


def test_mundo_check_avisa_de_la_regla_de_los_campos():
    a = avisos(suelto('{tipo: campo_escalones, pos: [0, 0], alturas: [[1, 4], [1, 2]]}'))
    assert len(a) == 1 and 'regla 1 de Jacoff' in a[0] and 'fila 1, columna 1' in a[0]


def test_mundo_check_avisa_del_cortado():
    a = avisos(suelto('{tipo: escalera, pos: [0, 0], ancho: 1, peldanos: 3,'
                      ' contrahuella: 0.1, huella: 0.3}'))
    assert len(a) == 1 and 'cortado de 0.30 m' in a[0]


def test_mundo_check_avisa_de_fuera_de_limites():
    texto = ('formato: 1\nlimites: [[0, 0], [2, 2]]\nelementos:\n'
             '  - {tipo: caja, pos: [0.5, 1], ancho: 1, largo: 1, alto: 0.1}\n'
             '  - {tipo: caja, pos: [1.5, 1], ancho: 1, largo: 1, alto: 0.1}\n')
    a = avisos(texto)
    assert a == ["elementos, elemento 2 (caja): fuera de 'limites'"]


def test_mundo_check_avisa_de_solapes():
    texto = ('formato: 1\ncarriles:\n  - {origen: [0, 0], ancho: 1, elementos: [\n'
             '      {tipo: caja, pos: [0, 0], largo: 1, alto: 0.1},\n'
             '      {tipo: caja, pos: [sigue, 0], largo: 1, alto: 0.2},\n'
             '      {tipo: caja, pos: [1.5, 0], largo: 1, alto: 0.1}]}\n')
    a = avisos(texto)
    assert a == ['carril_1, elemento 2 (caja): se solapa con el elemento 3 (caja)']


def test_ejemplos_sin_avisos_de_errata():
    for nombre in EJEMPLOS:
        assert not [a for a in mundo_check.avisos(mundo(nombre))
                    if re.search('solapa|limites|Jacoff|cortado', a)], nombre
