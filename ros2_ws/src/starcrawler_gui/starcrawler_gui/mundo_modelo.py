"""
mundo_modelo.py — los marcadores del mundo traducidos para el navegador
=======================================================================
Modulo PURO: no importa rclpy. Hermano de urdf_modelo.py: gui_node le pasa
los Marker de /mundo/marcadores y /mundo/indicadores, y la vista 3D dibuja
el JSON que sale. Asi la web pinta lo mismo que RViz, porque sale del
mismo sitio.

Acepta objetos con atributos: el Marker de ROS o SimpleNamespace en los
tests. Entiende TRIANGLE_LIST, LINE_LIST, LINE_STRIP, SPHERE_LIST, ARROW con
dos puntos y TEXT_VIEW_FACING. Los demas, los de otro marco y los que traen
numeros no finitos se cuentan en `omitidos`.

Tambien valida el JSON de /mundo/estado (validar_estado), que llega de
fuera y acaba en /events.
"""
from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

# Marker.type -> tipo de la pieza (el mismo nombre que en mundo_core.visual)
TIPOS = {11: 'triangulos', 5: 'lineas', 4: 'tira', 7: 'esferas',
         0: 'flecha', 9: 'texto'}

# Marker.action (ADD y MODIFY son el mismo valor)
ADD = 0
DELETE = 2
DELETEALL = 3

MAX_ESTADO = 4096           # bytes

_TEXTOS = ('mundo', 'modo', 'estado', 'motivo', 'consejo')
_NUMEROS = ('v', 'version', 'margen', 'cabeceo', 'balanceo', 'altura',
            'z_suelo', 'avance_orugas', 'avance_real', 'patinado', 'ms')
CLAVES_ESTADO = frozenset(_TEXTOS + _NUMEROS
                          + ('pieza', 'apoya', 'holgura', 'panza'))


# ─── Marcadores ──────────────────────────────────────────────────────────────

def _finitos(valores: List[float]) -> bool:
    return all(math.isfinite(v) for v in valores)


def _quat(q) -> List[float]:
    v = [float(q.x), float(q.y), float(q.z), float(q.w)]
    n = math.sqrt(sum(c * c for c in v))
    # RViz toma el cuaternion nulo por la identidad
    if not math.isfinite(n) or n < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [c / n for c in v]


def _colores(m, n_puntos: int, tipo: str) -> Optional[List[float]]:
    colores = list(m.colors or ())
    if not colores:
        return None
    if len(colores) == n_puntos:
        por_vertice = colores
    elif tipo == 'triangulos' and len(colores) * 3 == n_puntos:
        # Un color por cara, como admite RViz
        por_vertice = [c for c in colores for _ in range(3)]
    else:
        return None
    return [round(float(v), 3) for c in por_vertice
            for v in (c.r, c.g, c.b, c.a)]


def traducir(m, marco_fijo: str = 'odom') -> Optional[Dict]:
    """Un Marker (ADD) -> pieza, o None si no se puede dibujar."""
    tipo = TIPOS.get(int(m.type))
    if tipo is None:
        return None
    if str(m.header.frame_id).lstrip('/') != marco_fijo.lstrip('/'):
        return None

    pts = list(m.points or ())
    n = len(pts)
    if ((tipo == 'triangulos' and n % 3) or (tipo == 'lineas' and n % 2)
            or (tipo == 'flecha' and n != 2)):
        return None

    p = m.pose.position
    s = m.scale
    c = m.color
    pieza = {
        'clave': '%s/%d' % (m.ns, int(m.id)),
        'tipo': tipo,
        'pos': [float(p.x), float(p.y), float(p.z)],
        'quat': _quat(m.pose.orientation),
        'escala': [float(s.x), float(s.y), float(s.z)],
        'color': [round(float(v), 3) for v in (c.r, c.g, c.b, c.a)],
        'puntos': [round(float(v), 3) for q in pts for v in (q.x, q.y, q.z)],
        'colores': _colores(m, n, tipo),
        'texto': str(m.text) if tipo == 'texto' else '',
    }
    # Un NaN en el JSON rompe el JSON.parse de la pagina entera
    numeros = (pieza['pos'] + pieza['escala'] + pieza['color']
               + pieza['puntos'] + (pieza['colores'] or []))
    if not _finitos(numeros):
        return None
    return pieza


class Traductor:
    """Lo que hay dibujado, indexado por (ns, id), como en RViz."""

    def __init__(self, marco_fijo: str = 'odom'):
        self.marco_fijo = marco_fijo
        self.version = 0
        self._piezas: Dict[Tuple[str, int], Dict] = {}
        self._omitidos = set()

    def aplicar(self, markers) -> None:
        """ADD/MODIFY, DELETE y DELETEALL, en el orden del array."""
        for m in markers:
            accion = int(m.action)
            if accion == DELETEALL:
                self._piezas.clear()
                self._omitidos.clear()
                continue
            clave = (str(m.ns), int(m.id))
            if accion == DELETE:
                self._piezas.pop(clave, None)
                self._omitidos.discard(clave)
                continue
            pieza = traducir(m, self.marco_fijo) if accion == ADD else None
            if pieza is None:
                self._piezas.pop(clave, None)
                self._omitidos.add(clave)
            else:
                self._piezas[clave] = pieza
                self._omitidos.discard(clave)
        self.version += 1

    def a_json(self) -> Dict:
        return {'version': self.version,
                'piezas': list(self._piezas.values()),
                'omitidos': len(self._omitidos)}


def traducir_lista(markers, marco_fijo: str = 'odom') -> List[Dict]:
    """Para los indicadores: las piezas de un solo array, sin estado."""
    t = Traductor(marco_fijo)
    t.aplicar(markers)
    return t.a_json()['piezas']


# ─── /mundo/estado ───────────────────────────────────────────────────────────

def _numero(nombre: str, v) -> Optional[float]:
    if v is None:
        return None
    if type(v) not in (int, float):
        raise ValueError('%s: numero o null' % nombre)
    try:
        f = float(v)
    except OverflowError:
        raise ValueError('%s: numero fuera de rango' % nombre) from None
    # NaN o infinito: sin dato, que en /events romperia el JSON
    return f if math.isfinite(f) else None


def _texto(nombre: str, v) -> Optional[str]:
    if v is not None and type(v) is not str:
        raise ValueError('%s: texto o null' % nombre)
    return v


def _lista4(nombre: str, v, elemento) -> Optional[list]:
    if v is None:
        return None
    if type(v) is not list or len(v) != 4:
        raise ValueError('%s: lista de 4' % nombre)
    return [elemento(nombre, x) for x in v]


def _booleano(nombre: str, v) -> Optional[bool]:
    if v is not None and type(v) is not bool:
        raise ValueError('%s: true, false o null' % nombre)
    return v


def validar_estado(texto: str) -> Dict:
    """JSON de /mundo/estado (v 1) -> dict con solo las claves conocidas.

    ValueError si no es un objeto de la version 1, pasa de MAX_ESTADO o una
    clave conocida trae un tipo que no toca.
    """
    if len(texto.encode('utf-8')) > MAX_ESTADO:
        raise ValueError('mas de %d bytes' % MAX_ESTADO)
    try:
        datos = json.loads(texto)
    except ValueError:
        raise ValueError('JSON no valido') from None
    if type(datos) is not dict:
        raise ValueError('no es un objeto JSON')
    if datos.get('v') != 1 or type(datos.get('v')) is not int:
        raise ValueError('v: se esperaba 1')

    salida = {}
    for clave, v in datos.items():
        if clave not in CLAVES_ESTADO:
            continue
        if clave in _NUMEROS:
            salida[clave] = _numero(clave, v)
        elif clave in _TEXTOS or clave == 'pieza':
            salida[clave] = _texto(clave, v)
        elif clave == 'panza':
            salida[clave] = _booleano(clave, v)
        elif clave == 'apoya':
            salida[clave] = _lista4(clave, v, _booleano)
        else:
            salida[clave] = _lista4(clave, v, _numero)
    return salida
