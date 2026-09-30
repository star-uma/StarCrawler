"""
marcadores.py — lo que mundo_node cuenta del robot, sin ROS
===========================================================
Del Estado de contacto_core salen el JSON de /mundo/estado y los indicadores
de /mundo/indicadores (contactos, polígono de apoyo, CdG, choque y rótulo),
como dicts con el mismo contrato que mundo_core.Mundo.visual():

    {ns, id, tipo, puntos, colores, color, escala, texto, pos}

a_markers() convierte esos dicts en un MarkerArray, para el mundo y para los
indicadores, con un DELETEALL delante.

Módulo PURO: no importa rclpy, y los mensajes solo dentro de a_markers().
Probado en test/test_marcadores.py con un Estado falso.
"""
from __future__ import annotations

import json
import math
from typing import List, Optional, Sequence, Tuple

PIEZAS = ('FR', 'FL', 'RR', 'RL', 'chasis')
# De más grave a menos: manda el primero que se cumpla
PRIORIDAD = ('volcado', 'atrapado', 'cayendo', 'bloqueado', 'sin_traccion')

# Valores de Marker.type, para no importar visualization_msgs aquí
TIPO_MARKER = {'flecha': 0, 'tira': 4, 'lineas': 5, 'esferas': 7,
               'texto': 9, 'triangulos': 11}
CLAVES = ('ns', 'id', 'tipo', 'puntos', 'colores', 'color', 'escala',
          'texto', 'pos')

TOL_ACTIVO = 0.0005         # holgura de un contacto activo (tol_apoyo)
SOBRE_POLIGONO = 0.005
LARGO_CHOQUE = 0.15
SOBRE_CHASIS = 0.30
MARGEN_HOLGADO = 0.10
MARGEN_JUSTO = 0.03
VIDA_INDICADORES_S = 0.5


def _hex(c: str, alfa: float = 1.0) -> Tuple[float, float, float, float]:
    return (int(c[1:3], 16) / 255.0, int(c[3:5], 16) / 255.0,
            int(c[5:7], 16) / 255.0, alfa)


BLANCO = (1.0, 1.0, 1.0, 1.0)
GRIS = _hex('#c3c2b7')
AMBAR = _hex('#fab219')
ROJO = _hex('#d03b3b')

ESPEJO = ('sin bit 7: es el robot real o el driver serie',
          'el mundo solo funciona con sim:=true o con el ESP32 en HW_SIMULADO')

# En RViz el texto sale con una fuente sin acentos
ROTULOS = {'volcado': 'VOLCADO', 'atrapado': 'ATRAPADO', 'cayendo': 'CAYENDO',
           'bloqueado': 'BLOQUEADO', 'sin_traccion': 'SIN TRACCION'}


def clasificar(estado) -> str:
    """'volcado' | 'atrapado' | 'cayendo' | 'bloqueado' | 'sin_traccion' | 'libre'."""
    for clase in PRIORIDAD:
        if getattr(estado, clase):
            return clase
    return 'libre'


def textos(clase: str, pieza: Optional[str] = None,
           alto_cm: Optional[float] = None) -> Tuple[str, str]:
    """(motivo, consejo), con las etiquetas del DS4 y del mando web."""
    if clase == 'bloqueado':
        if pieza in ('FR', 'FL'):
            motivo = 'el morro choca'
            if alto_cm is not None:
                motivo += ' con un canto de %d cm' % round(alto_cm)
            return motivo, 'sube las delanteras (L1 · ▲ delanteras)'
        if pieza in ('RR', 'RL'):
            return 'la cola choca', 'sube las traseras (R1 · ▲ traseras)'
        return ('el chasis choca',
                'baja los cuatro brazos para levantar el chasis (L2 + R2)')
    if clase == 'sin_traccion':
        return 'apoya la panza y las orugas no tocan', 'baja los brazos (L2 / R2)'
    if clase == 'volcado':
        return ('más de 75° de inclinación',
                'reinicia: /mundo/reiniciar o 2D Pose Estimate en RViz')
    if clase == 'atrapado':
        return 'empujado por los dos lados', 'mueve los brazos o retrocede'
    if clase == 'cayendo':
        return 'gira sobre un canto hasta apoyar', 'espera a que se asiente'
    return '', ''


def contacto_que_bloquea(estado):
    """El contacto de la pieza bloqueada que frena: el de pared más alto."""
    if not estado.bloqueado:
        return None
    suyos = [c for c in estado.contactos if c.pieza == estado.bloqueado]
    if not suyos:
        return None
    paredes = [c for c in suyos if c.tipo == 'pared']
    return max(paredes or suyos, key=lambda c: c.z)


def dentro_del_choque(contacto, d: float = 0.01) -> Optional[Tuple[float, float]]:
    """Punto en planta a d del contacto, hacia el obstáculo (contra la normal)."""
    nx, ny = contacto.normal[0], contacto.normal[1]
    h = math.hypot(nx, ny)
    if h < 1e-6:
        return None
    return contacto.x - d * nx / h, contacto.y - d * ny / h


def _num(v, decimales: int):
    """Redondeado, y None si no es finito: NaN no es JSON."""
    if v is None or not math.isfinite(v):
        return None
    return round(float(v), decimales)


def estado_json(estado, mundo: str = '', version: int = 0, modo: str = 'normal',
                z_suelo: float = 0.0, avance_orugas: float = 0.0,
                avance_real: float = 0.0, patinado: float = 0.0, ms: float = 0.0,
                alto_choque: Optional[float] = None,
                aviso: Optional[Tuple[str, str]] = None) -> str:
    """El JSON versión 1 de /mundo/estado.

    alto_choque: alto del obstáculo que bloquea sobre z_suelo, en m; si no se
    da, la z del contacto. aviso: (motivo, consejo) para cuando está libre,
    por ejemplo un YAML que no carga.
    """
    clase = clasificar(estado)
    if alto_choque is None:
        c = contacto_que_bloquea(estado)
        if c is not None:
            alto_choque = c.z - z_suelo
    if modo == 'espejo':
        motivo, consejo = ESPEJO
    elif clase == 'libre' and aviso:
        motivo, consejo = aviso
    else:
        motivo, consejo = textos(
            clase, estado.bloqueado,
            None if alto_choque is None else 100.0 * alto_choque)
    p = estado.pose
    datos = {
        'v': 1, 'mundo': mundo, 'version': int(version), 'modo': modo,
        'estado': clase, 'pieza': estado.bloqueado or None,
        'motivo': motivo, 'consejo': consejo,
        'apoya': [bool(a) for a in estado.apoya],
        'holgura': [_num(h, 4) for h in estado.holguras],
        'panza': bool(estado.panza),
        'margen': _num(estado.margen, 4),
        'cabeceo': _num(p.cabeceo, 5), 'balanceo': _num(p.balanceo, 5),
        'altura': _num(p.z, 4), 'z_suelo': _num(z_suelo, 4),
        'avance_orugas': _num(avance_orugas, 4),
        'avance_real': _num(avance_real, 4),
        'patinado': _num(patinado, 4), 'ms': _num(ms, 2),
    }
    return json.dumps(datos, ensure_ascii=False, separators=(',', ':'))


def color_margen(margen) -> Tuple[float, float, float, float]:
    if margen is None or not math.isfinite(margen) or margen < MARGEN_JUSTO:
        return ROJO
    return GRIS if margen > MARGEN_HOLGADO else AMBAR


def _pieza(ns, tipo, puntos, color, escala, texto=None, pos=None) -> dict:
    return {'ns': ns, 'id': 0, 'tipo': tipo,
            'puntos': [tuple(float(v) for v in q) for q in puntos],
            'colores': None, 'color': color, 'escala': escala,
            'texto': texto, 'pos': pos}


def indicadores(estado) -> List[dict]:
    """Los indicadores de /mundo/indicadores, todos en el marco odom."""
    activos = [c for c in estado.contactos
               if c.tipo == 'apoyo' and c.holgura <= TOL_ACTIVO]
    bandas = [(c.x, c.y, c.z) for c in activos if c.pieza != 'chasis']
    panza = [(c.x, c.y, c.z) for c in activos if c.pieza == 'chasis']
    color = color_margen(estado.margen)
    cdg = tuple(float(v) for v in estado.cdg)
    salida = []
    if bandas:
        salida.append(_pieza('contactos', 'esferas', bandas, BLANCO, 0.035))
    if panza:
        salida.append(_pieza('panza', 'esferas', panza, AMBAR, 0.035))
    poligono = [(x, y, z + SOBRE_POLIGONO) for x, y, z in estado.poligono]
    if len(poligono) >= 2:
        salida.append(_pieza('poligono', 'tira', poligono + poligono[:1],
                             color, 0.012))
    salida.append(_pieza('cdg', 'esferas', [cdg], color, 0.045))
    cotas = [c.z for c in activos] or [z for _, _, z in estado.poligono]
    if cotas:
        salida.append(_pieza('plomada', 'lineas',
                             [cdg, (cdg[0], cdg[1], min(cotas))], color, 0.004))
    choque = contacto_que_bloquea(estado)
    if choque is not None:
        nx, ny = choque.normal[0], choque.normal[1]
        h = math.hypot(nx, ny)
        if h > 1e-6:
            origen = (choque.x, choque.y, choque.z)
            punta = (choque.x + LARGO_CHOQUE * nx / h,
                     choque.y + LARGO_CHOQUE * ny / h, choque.z)
            salida.append(_pieza('choque', 'flecha', [origen, punta], AMBAR, 0.02))
    clase = clasificar(estado)
    if clase != 'libre':
        p = estado.pose
        rotulo = ROTULOS[clase]
        if clase == 'bloqueado':
            rotulo += ' (%s)' % estado.bloqueado
        salida.append(_pieza(
            'estado', 'texto', [], ROJO if clase == 'volcado' else AMBAR, 0.08,
            texto=rotulo, pos=(float(p.x), float(p.y), float(p.z) + SOBRE_CHASIS)))
    return salida


def cuaternion_zyx(yaw: float, cabeceo: float,
                   balanceo: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) de Rz(yaw)·Ry(cabeceo)·Rx(balanceo), el orden de las juntas."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(cabeceo / 2.0), math.sin(cabeceo / 2.0)
    cr, sr = math.cos(balanceo / 2.0), math.sin(balanceo / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _comprobar(p: dict) -> None:
    donde = '%s/%s' % (p.get('ns'), p.get('id'))
    tipo = p.get('tipo')
    if tipo not in TIPO_MARKER:
        raise ValueError('%s: tipo %r desconocido' % (donde, tipo))
    n = len(p.get('puntos') or ())
    if ((tipo == 'triangulos' and n % 3) or (tipo == 'lineas' and n % 2)
            or (tipo == 'flecha' and n != 2)):
        raise ValueError('%s: %d puntos no valen para %s' % (donde, n, tipo))
    colores = p.get('colores')
    if colores is not None and len(colores) != n:
        raise ValueError('%s: %d colores para %d puntos' % (donde, len(colores), n))


def a_markers(piezas: Sequence[dict], marco: str, sello,
              vida_s: float = 0.0):
    """dicts de visual() o indicadores() -> MarkerArray, con DELETEALL delante.

    sello es un builtin_interfaces/Time. Un dict fuera del contrato da
    ValueError, sin publicar nada.
    """
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    def rgba(c):
        return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]),
                         a=float(c[3]))

    for p in piezas:
        _comprobar(p)
    salida = MarkerArray()
    borrar = Marker()
    borrar.header.frame_id = marco
    borrar.header.stamp = sello
    borrar.action = Marker.DELETEALL
    salida.markers.append(borrar)
    ns_vida = int(round(vida_s * 1e9))
    vida = Duration(sec=ns_vida // 10 ** 9, nanosec=ns_vida % 10 ** 9)
    for p in piezas:
        m = Marker()
        m.header.frame_id = marco
        m.header.stamp = sello
        m.ns = str(p['ns'])
        m.id = int(p['id'])
        m.type = TIPO_MARKER[p['tipo']]
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        escala = float(p['escala'])
        if p['tipo'] == 'texto':
            x, y, z = p['pos'] or (0.0, 0.0, 0.0)
            m.pose.position = Point(x=float(x), y=float(y), z=float(z))
            m.text = p['texto'] or ''
            m.scale.z = escala
        else:
            m.points = [Point(x=float(x), y=float(y), z=float(z))
                        for x, y, z in p['puntos']]
            if p['tipo'] == 'triangulos':
                m.scale.x = m.scale.y = m.scale.z = 1.0
            elif p['tipo'] == 'esferas':
                m.scale.x = m.scale.y = m.scale.z = escala
            elif p['tipo'] == 'flecha':
                m.scale.x, m.scale.y, m.scale.z = 0.02, 0.04, 0.05
            else:
                m.scale.x = escala
        m.color = rgba(p['color'])
        if p.get('colores'):
            m.colors = [rgba(c) for c in p['colores']]
        m.lifetime = vida
        salida.markers.append(m)
    return salida
