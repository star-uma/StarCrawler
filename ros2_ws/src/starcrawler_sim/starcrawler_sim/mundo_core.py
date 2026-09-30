"""
mundo_core.py — el mundo con obstaculos, sin ROS
================================================
Modulo PURO: no importa rclpy. Lee un mundo en YAML (formato 1) y lo reduce
a SOLIDOS: rectangulos orientados con techo plano, macizos hasta z = 0, de
clase terreno o pared. Sobre ellos responde altura(x, y) y perfil(), que es
lo que usa contacto_core, y visual(), la geometria que mundo_node publica
como MarkerArray.

Todo lo que sale de aqui esta en odom: la pose 'inicio' del fichero es el
origen de odom. El formato se ve en mundos/practica_rrl.yaml.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

FORMATO = 1

CELDA_INDICE = 0.25
MARGEN_INDICE = 1e-6
EPS_BORDE = 1e-9            # los bordes de los solidos son cerrados
TOL_S = 1e-12               # dos rupturas mas cerca que esto son la misma
SALTO_MIN = 1e-9            # por encima, dos vertices con la misma s
TECHO_MIN = 1e-3            # caras laterales y aristas solo por encima
MAX_VERTICES = 60000

ALTO_PARED = 0.8
GROSOR_PARED = 0.02

GRIS_SUELO = (0x6b / 255, 0x67 / 255, 0x5f / 255)
GRIS_ALTO = (0xcd / 255, 0xc6 / 255, 0xb6 / 255)
Z_GRIS_ALTO = 1.0
COLOR_TERRENO = (1.0, 1.0, 1.0, 1.0)
COLOR_PARED = (0.478, 0.510, 0.549, 0.28)
COLOR_ARISTA = (0.11, 0.11, 0.10, 0.7)
ESCALA_ARISTA = 0.004
COLOR_ZONA = (1.0, 1.0, 1.0, 0.5)
ESCALA_ZONA = 0.01
Z_ZONA = 0.002
COLOR_ETIQUETA = (0.76, 0.76, 0.72, 1.0)
ESCALA_ETIQUETA = 0.08
Z_ETIQUETA_CARRIL = 0.4
Z_ETIQUETA_ZONA = 0.3

CLAVES_MUNDO = ('formato', 'nombre', 'descripcion', 'inicio', 'limites',
                'paredes', 'zonas', 'carriles', 'elementos')
CLAVES_INICIO = ('x', 'y', 'rumbo')
CLAVES_PARED = ('puntos', 'alto', 'grosor')
CLAVES_ZONA = ('nombre', 'de', 'a')
CLAVES_CARRIL = ('nombre', 'titulo', 'origen', 'rumbo', 'largo', 'ancho',
                 'color', 'elementos')
CLAVES_TIPO = {
    'caja': ('pos', 'rumbo', 'ancho', 'largo', 'alto'),
    'rampa': ('pos', 'rumbo', 'ancho', 'z', 'largo', 'alto', 'pendiente'),
    'escalera': ('pos', 'rumbo', 'ancho', 'z', 'peldanos', 'contrahuella',
                 'huella', 'pendiente', 'rellano', 'bajada'),
    'campo_escalones': ('pos', 'celda', 'unidad', 'base', 'alturas', 'patron',
                        'filas', 'columnas'),
    'campo_rampas': ('pos', 'celda', 'pendiente', 'orientaciones', 'patron',
                     'filas', 'columnas'),
    'viga': ('de', 'a', 'seccion', 'alto'),
}
TIPOS_FUERA = ('tubo', 'cilindro', 'zanja', 'hueco')
CLAVES_FUERA = ('encima', 'tubo', 'tubo_salida', 'color_tubo', 'agarre')

# Hacia donde sube cada celda de un campo de rampas: (ds, dt) unitario
SUBIDA = {'A': (1.0, 0.0), 'T': (-1.0, 0.0), 'I': (0.0, 1.0), 'D': (0.0, -1.0)}


class ErrorMundo(ValueError):
    pass


def _cs(ang: float) -> Tuple[float, float]:
    """cos y sin, exactos en los multiplos de 90 grados."""
    k = round(ang / (math.pi / 2))
    if abs(ang - k * math.pi / 2) < 1e-12:
        return ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))[k % 4]
    return math.cos(ang), math.sin(ang)


def _normalizar(ang: float) -> float:
    return math.atan2(math.sin(ang), math.cos(ang))


def _gris(z: float) -> Tuple[float, float, float, float]:
    f = min(1.0, max(0.0, z / Z_GRIS_ALTO))
    return tuple(b + (a - b) * f for a, b in zip(GRIS_ALTO, GRIS_SUELO)) + (1.0,)


# ============================================================== solidos

@dataclass(frozen=True)
class Solido:
    cx: float
    cy: float
    rumbo: float                # rad en odom: la direccion del largo
    largo: float
    ancho: float
    a: float                    # techo z = a + bx*x + by*y, con (x, y) en odom
    bx: float
    by: float
    clase: str = 'terreno'      # 'terreno' | 'pared'
    color: Optional[Tuple[float, float, float]] = None
    origen: str = ''

    def techo(self, x: float, y: float) -> float:
        return self.a + self.bx * x + self.by * y

    def esquinas(self) -> List[Tuple[float, float]]:
        """Las cuatro esquinas en planta, antihorarias."""
        c, s = _cs(self.rumbo)
        hl, hw = self.largo / 2, self.ancho / 2
        return [(self.cx + c * u - s * v, self.cy + s * u + c * v)
                for u, v in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]

    def contiene(self, x: float, y: float) -> bool:
        c, s = _cs(self.rumbo)
        dx, dy = x - self.cx, y - self.cy
        return (abs(c * dx + s * dy) <= self.largo / 2 + EPS_BORDE
                and abs(-s * dx + c * dy) <= self.ancho / 2 + EPS_BORDE)


@dataclass(frozen=True)
class Zona:
    nombre: str
    esquinas: Tuple[Tuple[float, float], ...]      # antihorarias, en odom
    centro: Tuple[float, float]


@dataclass(frozen=True)
class Carril:
    nombre: str
    titulo: str
    origen: Tuple[float, float]                    # centro de la entrada, en odom
    rumbo: float                                   # rad en odom
    largo: Optional[float]
    ancho: Optional[float]
    color: Optional[Tuple[float, float, float]]
    s_min: float                                   # tramo ocupado por sus solidos
    s_max: float
    z_max: float
    solidos: Tuple[int, ...]


@dataclass(frozen=True)
class Elemento:
    """De donde sale cada grupo de solidos; lo usa mundo_check."""
    grupo: str
    indice: int
    tipo: str
    solidos: Tuple[int, ...]
    matriz: Optional[Tuple[Tuple[int, ...], ...]] = None    # campo_escalones
    cortado: float = 0.0                                    # escalera sin bajada

    @property
    def donde(self) -> str:
        return '%s, elemento %d (%s)' % (self.grupo, self.indice, self.tipo)


# ============================================================== el mundo

class Mundo:
    def __init__(self, nombre: str = 'vacio', solidos: Sequence[Solido] = (),
                 zonas: Sequence[Zona] = (), carriles: Sequence[Carril] = (),
                 inicio: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 descripcion: str = '', limites=None,
                 elementos: Sequence[Elemento] = (), avisos: Sequence[str] = ()):
        self.nombre = nombre
        self.descripcion = descripcion
        self.inicio = tuple(inicio)     # (x, y, rumbo en rad) del fichero = origen de odom
        self.limites = limites          # en coordenadas del fichero
        self.solidos = tuple(solidos)
        self.zonas = tuple(zonas)
        self.carriles = tuple(carriles)
        self.elementos = tuple(elementos)
        self.avisos = list(avisos)
        self._celda = CELDA_INDICE
        self._indexar()

    # ---------------------------------------------------------- marcos
    def a_odom(self, x: float, y: float) -> Tuple[float, float]:
        xi, yi, ri = self.inicio
        c, s = _cs(-ri)
        dx, dy = x - xi, y - yi
        return c * dx - s * dy, s * dx + c * dy

    def a_fichero(self, x: float, y: float) -> Tuple[float, float]:
        xi, yi, ri = self.inicio
        c, s = _cs(ri)
        return xi + c * x - s * y, yi + s * x + c * y

    # ---------------------------------------------------------- indice
    def _indexar(self) -> None:
        self._geo = []
        self._indice: Dict[Tuple[int, int], List[int]] = {}
        C = self._celda
        for k, so in enumerate(self.solidos):
            c, s = _cs(so.rumbo)
            self._geo.append((so.cx, so.cy, c, s, so.largo / 2, so.ancho / 2,
                              so.a, so.bx, so.by, so.clase == 'pared'))
            e = so.esquinas()
            xs = [p[0] for p in e]
            ys = [p[1] for p in e]
            for i in range(math.floor((min(xs) - MARGEN_INDICE) / C),
                           math.floor((max(xs) + MARGEN_INDICE) / C) + 1):
                for j in range(math.floor((min(ys) - MARGEN_INDICE) / C),
                               math.floor((max(ys) + MARGEN_INDICE) / C) + 1):
                    self._indice.setdefault((i, j), []).append(k)

    def _celdas(self, xa: float, ya: float, xb: float, yb: float):
        """Celdas que recorre el segmento (DDA de Amanatides y Woo)."""
        C = self._celda
        i, j = math.floor(xa / C), math.floor(ya / C)
        n = abs(math.floor(xb / C) - i) + abs(math.floor(yb / C) - j)
        celdas = [(i, j)]
        if n == 0:
            return celdas
        dx, dy = xb - xa, yb - ya
        inf = math.inf
        if dx > 0:
            si, tx, ddx = 1, ((i + 1) * C - xa) / dx, C / dx
        elif dx < 0:
            si, tx, ddx = -1, (i * C - xa) / dx, -C / dx
        else:
            si, tx, ddx = 0, inf, inf
        if dy > 0:
            sj, ty, ddy = 1, ((j + 1) * C - ya) / dy, C / dy
        elif dy < 0:
            sj, ty, ddy = -1, (j * C - ya) / dy, -C / dy
        else:
            sj, ty, ddy = 0, inf, inf
        for _ in range(n):
            if tx < ty:
                i += si
                tx += ddx
            else:
                j += sj
                ty += ddy
            celdas.append((i, j))
        return celdas

    # ---------------------------------------------------------- consultas
    def altura(self, x: float, y: float, paredes: bool = True) -> float:
        """max(0, techos de los solidos que contienen el punto), bordes incluidos."""
        C = self._celda
        lista = self._indice.get((math.floor(x / C), math.floor(y / C)))
        z = 0.0
        if lista:
            geo = self._geo
            for k in lista:
                cx, cy, c, s, hl, hw, a, bx, by, pared = geo[k]
                if pared and not paredes:
                    continue
                dx, dy = x - cx, y - cy
                u = c * dx + s * dy
                if u > hl + EPS_BORDE or u < -hl - EPS_BORDE:
                    continue
                v = c * dy - s * dx
                if v > hw + EPS_BORDE or v < -hw - EPS_BORDE:
                    continue
                h = a + bx * x + by * y
                if h > z:
                    z = h
        return z

    def perfil(self, x0: float, y0: float, ux: float, uy: float,
               s0: float, s1: float) -> List[Tuple[float, float]]:
        """Polilinea (s, z) del terreno a lo largo de (x0, y0) + s*(ux, uy).

        s no decrece; un salto vertical son dos vertices con la misma s,
        primero el valor por la izquierda y despues el de la derecha.
        """
        if s1 < s0:
            raise ValueError('perfil: s1 < s0')
        if not self._geo:
            return [(s0, 0.0), (s1, 0.0)]
        ind = self._indice
        cand = set()
        for celda in self._celdas(x0 + s0 * ux, y0 + s0 * uy,
                                  x0 + s1 * ux, y0 + s1 * uy):
            lista = ind.get(celda)
            if lista:
                cand.update(lista)

        # Cada candidato, recortado con la recta (slabs): z lineal en [lo, hi]
        lineas = []
        geo = self._geo
        for k in cand:
            cx, cy, c, s, hl, hw, a, bx, by, _ = geo[k]
            dx, dy = x0 - cx, y0 - cy
            lo, hi = s0, s1
            du = c * ux + s * uy
            u0 = c * dx + s * dy
            if -1e-12 < du < 1e-12:
                if u0 > hl + EPS_BORDE or u0 < -hl - EPS_BORDE:
                    continue
            else:
                t1, t2 = (-hl - u0) / du, (hl - u0) / du
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > lo:
                    lo = t1
                if t2 < hi:
                    hi = t2
                if hi - lo <= TOL_S:
                    continue
            dv = c * uy - s * ux
            v0 = c * dy - s * dx
            if -1e-12 < dv < 1e-12:
                if v0 > hw + EPS_BORDE or v0 < -hw - EPS_BORDE:
                    continue
            else:
                t1, t2 = (-hw - v0) / dv, (hw - v0) / dv
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > lo:
                    lo = t1
                if t2 < hi:
                    hi = t2
                if hi - lo <= TOL_S:
                    continue
            lineas.append((lo, hi, a + bx * x0 + by * y0, bx * ux + by * uy))
        if not lineas:
            return [(s0, 0.0), (s1, 0.0)]

        # Rupturas: extremos de los intervalos y cruces con las inclinadas
        suelo = (s0, s1, 0.0, 0.0)
        rupturas = [s0, s1]
        for lo, hi, _, _ in lineas:
            rupturas.append(lo)
            rupturas.append(hi)
        todas = lineas + [suelo]
        for lo1, hi1, p1, m1 in todas:
            if m1 == 0.0:
                continue
            for lo2, hi2, p2, m2 in todas:
                dm = m1 - m2
                if -1e-12 < dm < 1e-12:
                    continue
                sc = (p2 - p1) / dm
                if max(lo1, lo2) + TOL_S < sc < min(hi1, hi2) - TOL_S:
                    rupturas.append(sc)
        rupturas.sort()
        rs = [rupturas[0]]
        for r in rupturas:
            if r - rs[-1] > TOL_S:
                rs.append(r)
        rs[-1] = s1
        if len(rs) == 1:
            rs.append(s1)

        # Envolvente superior: entre dos rupturas manda una sola recta
        lineas.sort()
        activas = [suelo]
        j, nl = 0, len(lineas)
        pts: List[Tuple[float, float]] = []
        for k in range(len(rs) - 1):
            ra, rb = rs[k], rs[k + 1]
            while j < nl and lineas[j][0] <= ra + TOL_S:
                activas.append(lineas[j])
                j += 1
            activas = [r for r in activas if r[1] >= rb - TOL_S]
            medio = 0.5 * (ra + rb)
            mejor = suelo
            zm = -math.inf
            for r in activas:
                z = r[2] + r[3] * medio
                if z > zm:
                    zm, mejor = z, r
            p, m = mejor[2], mejor[3]
            # El suelo siempre esta: fuera el -1e-17 de una rampa que acaba en 0
            zi, zd = max(0.0, p + m * ra), max(0.0, p + m * rb)
            if not pts or abs(pts[-1][1] - zi) > SALTO_MIN:
                pts.append((ra, zi))
            pts.append((rb, zd))

        # Fuera los vertices colineales
        res = [pts[0]]
        for k in range(1, len(pts) - 1):
            sa, za = res[-1]
            sb, zb = pts[k]
            sc, zc = pts[k + 1]
            if (sb - sa > TOL_S and sc - sb > TOL_S
                    and abs(za + (zc - za) * (sb - sa) / (sc - sa) - zb) <= 1e-10):
                continue
            res.append(pts[k])
        res.append(pts[-1])
        return res

    # ---------------------------------------------------------- vistas
    def visual(self) -> List[dict]:
        """Geometria para mundo_node (dicts planos, §4.3); las piezas vacias no salen."""
        tri: List[Tuple[float, float, float]] = []
        col: List[Tuple[float, float, float, float]] = []
        par: List[Tuple[float, float, float]] = []
        ari: List[Tuple[float, float, float]] = []
        for so in self.solidos:
            e = so.esquinas()
            arriba = [(x, y, max(0.0, so.techo(x, y))) for x, y in e]
            abajo = [(x, y, 0.0) for x, y in e]
            if so.clase == 'pared':
                par.extend(_caras(arriba, abajo))
                continue
            caras = _caras(arriba, abajo)
            tri.extend(caras)
            if so.color is not None:
                col.extend([tuple(so.color) + (1.0,)] * len(caras))
            else:
                col.extend(_gris(p[2]) for p in caras)
            if max(p[2] for p in arriba) > TECHO_MIN:
                for k in range(4):
                    ari.extend((arriba[k], arriba[(k + 1) % 4]))
                    if arriba[k][2] > TECHO_MIN:
                        ari.extend((abajo[k], arriba[k]))

        out: List[dict] = []
        if tri:
            out.append(_pieza('terreno', 0, 'triangulos', tri, COLOR_TERRENO, 1.0, col))
        if par:
            out.append(_pieza('paredes', 0, 'triangulos', par, COLOR_PARED, 1.0))
        if ari:
            out.append(_pieza('aristas', 0, 'lineas', ari, COLOR_ARISTA, ESCALA_ARISTA))
        for k, zo in enumerate(self.zonas):
            contorno = [(x, y, Z_ZONA) for x, y in zo.esquinas]
            contorno.append(contorno[0])
            out.append(_pieza('zonas', k, 'tira', contorno, COLOR_ZONA, ESCALA_ZONA))
        k = 0
        for ca in self.carriles:
            out.append(_texto(k, ca.titulo, (ca.origen[0], ca.origen[1],
                                             ca.z_max + Z_ETIQUETA_CARRIL)))
            k += 1
        for zo in self.zonas:
            out.append(_texto(k, zo.nombre, (zo.centro[0], zo.centro[1], Z_ETIQUETA_ZONA)))
            k += 1

        n = sum(len(d['puntos']) for d in out)
        if n > MAX_VERTICES:
            aviso = 'la malla tiene %d vertices (mas de %d)' % (n, MAX_VERTICES)
            if aviso not in self.avisos:
                self.avisos.append(aviso)
        return out

    def resumen(self) -> str:
        vis = self.visual()
        ntri = sum(len(d['puntos']) // 3 for d in vis if d['tipo'] == 'triangulos')
        terreno = [so for so in self.solidos if so.clase == 'terreno']
        zmax = max((_z_max(so) for so in terreno), default=0.0)
        lineas = ['mundo %s: %d solidos (%d de pared), %d triangulos, altura maxima %.3f m'
                  % (self.nombre, len(self.solidos), len(self.solidos) - len(terreno),
                     ntri, zmax)]
        for ca in self.carriles:
            if ca.solidos:
                lineas.append('  %s: s de %.3f a %.3f m, altura maxima %.3f m (%d solidos)'
                              % (ca.nombre, ca.s_min, ca.s_max, ca.z_max, len(ca.solidos)))
            else:
                lineas.append('  %s: vacio' % ca.nombre)
        sueltos = sum(len(e.solidos) for e in self.elementos if e.grupo == 'elementos')
        if sueltos:
            lineas.append('  elementos sueltos: %d solidos' % sueltos)
        for a in self.avisos:
            lineas.append('  aviso: ' + a)
        return '\n'.join(lineas)


def _z_max(so: Solido) -> float:
    return max(so.techo(x, y) for x, y in so.esquinas())


def _caras(arriba, abajo) -> List[Tuple[float, float, float]]:
    """Triangulos antihorarios vistos desde fuera: tapa y laterales hasta z = 0."""
    out: List[Tuple[float, float, float]] = []
    _triangulo(out, arriba[0], arriba[1], arriba[2])
    _triangulo(out, arriba[0], arriba[2], arriba[3])
    for k in range(4):
        m = (k + 1) % 4
        if max(arriba[k][2], arriba[m][2]) > TECHO_MIN:
            _triangulo(out, abajo[k], abajo[m], arriba[m])
            _triangulo(out, abajo[k], arriba[m], arriba[k])
    return out


def _triangulo(out, p0, p1, p2) -> None:
    ax, ay, az = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    bx, by, bz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
    nx, ny, nz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
    if nx * nx + ny * ny + nz * nz > 1e-20:
        out.extend((p0, p1, p2))


def _pieza(ns, id_, tipo, puntos, color, escala, colores=None) -> dict:
    return {'ns': ns, 'id': id_, 'tipo': tipo, 'puntos': list(puntos),
            'colores': list(colores) if colores is not None else None,
            'color': tuple(color), 'escala': escala, 'texto': None, 'pos': None}


def _texto(id_, texto, pos) -> dict:
    return {'ns': 'etiquetas', 'id': id_, 'tipo': 'texto', 'puntos': [],
            'colores': None, 'color': COLOR_ETIQUETA, 'escala': ESCALA_ETIQUETA,
            'texto': texto, 'pos': tuple(pos)}


# ============================================================== lectura

def _numero(d: dict, clave: str, donde: str, defecto=None) -> float:
    if clave not in d:
        if defecto is None:
            raise ErrorMundo("%s: falta '%s'" % (donde, clave))
        return float(defecto)
    return _valor(d[clave], clave, donde)


def _valor(v, clave: str, donde: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ErrorMundo("%s: '%s' debe ser un numero finito (es %r)" % (donde, clave, v))
    return float(v)


def _positivo(d: dict, clave: str, donde: str, defecto=None) -> float:
    v = _numero(d, clave, donde, defecto)
    if v <= 0:
        raise ErrorMundo("%s: '%s' debe ser > 0 (es %g)" % (donde, clave, v))
    return v


def _cota(d: dict, clave: str, donde: str, defecto=0.0) -> float:
    v = _numero(d, clave, donde, defecto)
    if v < 0:
        raise ErrorMundo("%s: '%s' debe ser >= 0 (es %g)" % (donde, clave, v))
    return v


def _angulo_agudo(d: dict, clave: str, donde: str, defecto=None) -> float:
    """Pendiente en grados, en (0, 90); devuelve radianes."""
    v = _numero(d, clave, donde, defecto)
    if not 0 < v < 90:
        raise ErrorMundo("%s: '%s' debe estar entre 0 y 90 grados (es %g)"
                         % (donde, clave, v))
    return math.radians(v)


def _entero(d: dict, clave: str, donde: str, minimo: int = 1) -> int:
    if clave not in d:
        raise ErrorMundo("%s: falta '%s'" % (donde, clave))
    v = d[clave]
    if isinstance(v, bool) or not isinstance(v, int) or v < minimo:
        raise ErrorMundo("%s: '%s' debe ser un entero >= %d (es %r)"
                         % (donde, clave, minimo, v))
    return v


def _punto(v, clave: str, donde: str) -> Tuple[float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise ErrorMundo("%s: '%s' debe ser [x, y] (es %r)" % (donde, clave, v))
    return _valor(v[0], clave, donde), _valor(v[1], clave, donde)


def _texto_de(d: dict, clave: str, donde: str, defecto: str) -> str:
    v = d.get(clave, defecto)
    if not isinstance(v, str):
        raise ErrorMundo("%s: '%s' debe ser un texto (es %r)" % (donde, clave, v))
    return v


def _lista(d: dict, clave: str, donde: str) -> list:
    v = d.get(clave)
    if v is None:
        return []
    if not isinstance(v, list):
        raise ErrorMundo("%s: '%s' debe ser una lista" % (donde, clave))
    return v


def _mapa(v, donde: str) -> dict:
    if not isinstance(v, dict):
        raise ErrorMundo('%s: debe ser un diccionario {clave: valor} (es %r)' % (donde, v))
    return v


def _color(d: dict, donde: str) -> Optional[Tuple[float, float, float]]:
    if 'color' not in d:
        return None
    v = d['color']
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ErrorMundo("%s: 'color' debe ser [r, g, b] (es %r)" % (donde, v))
    c = tuple(_valor(x, 'color', donde) for x in v)
    if any(x < 0 or x > 1 for x in c):
        raise ErrorMundo("%s: 'color' va de 0 a 1 (es %r)" % (donde, v))
    return c


def _claves(d: dict, permitidas: Sequence[str], donde: str, que: str = '') -> None:
    for k in d:
        if k in permitidas:
            continue
        if k in CLAVES_FUERA:
            raise ErrorMundo("%s: '%s' no soportado en formato 1" % (donde, k))
        if que and any(k in v for v in CLAVES_TIPO.values()):
            raise ErrorMundo("%s: '%s' no se admite en %s" % (donde, k, que))
        raise ErrorMundo("%s: clave desconocida '%s'" % (donde, k))


def _matriz(v, clave: str, donde: str) -> List[list]:
    if (not isinstance(v, list) or not v
            or not all(isinstance(f, list) and f for f in v)):
        raise ErrorMundo("%s: '%s' debe ser una matriz (lista de filas)" % (donde, clave))
    if any(len(f) != len(v[0]) for f in v):
        raise ErrorMundo("%s: '%s' no es rectangular (filas de %s celdas)"
                         % (donde, clave, sorted({len(f) for f in v})))
    return v


def _dos_de(d: dict, claves: Sequence[str], n: int, donde: str, que: str) -> List[str]:
    tiene = [k for k in claves if k in d]
    if len(tiene) != n:
        raise ErrorMundo('%s: %s lleva exactamente %s de %s (tiene %s)'
                         % (donde, que, {1: 'una', 2: 'dos'}[n], '/'.join(claves),
                            '/'.join(tiene) or 'ninguna'))
    return tiene


def patron_escalones(nombre, filas: int, columnas: int, donde: str) -> List[List[int]]:
    """Patrones del prototipo, inspirados en ASTM E2828 (no son la norma)."""
    if nombre == 'colina_diagonal':
        perfil = [4, 3, 3, 2, 2, 1]
        return [[perfil[min(abs(i - j), 5)] for j in range(columnas)]
                for i in range(filas)]
    if nombre == 'plano_cruz':
        if filas < 5 or columnas < 5:
            raise ErrorMundo('%s: plano_cruz necesita al menos 5 filas y 5 columnas' % donde)
        ci, cj = (filas - 1) / 2, (columnas - 1) / 2
        out = [[2 if (abs(i - ci) < 1 or abs(j - cj) < 1) else 1
                for j in range(columnas)] for i in range(filas)]
        for i, j in ((2, 2), (2, columnas - 3), (filas - 3, 2), (filas - 3, columnas - 3)):
            out[i][j] = 3
        return out
    raise ErrorMundo("%s: patron '%s' desconocido (plano_cruz o colina_diagonal)"
                     % (donde, nombre))


def patron_rampas(nombre, filas: int, columnas: int, donde: str) -> List[List[str]]:
    if nombre == 'cruzadas':
        return [['A' if (i + j) % 2 == 0 else 'I' for j in range(columnas)]
                for i in range(filas)]
    if nombre == 'continuas':
        return [['A' if i % 2 == 0 else 'T'] * columnas for i in range(filas)]
    raise ErrorMundo("%s: patron '%s' desconocido (cruzadas o continuas)" % (donde, nombre))


class _Grupo:
    """Marco de un carril (o de los elementos sueltos): s a lo largo, t a la izquierda."""

    def __init__(self, nombre, ox, oy, rumbo, ancho, color):
        self.nombre = nombre
        self.ox, self.oy, self.rumbo = ox, oy, rumbo
        self.ancho = ancho
        self.color = color
        self.fin: Optional[float] = None    # donde acabo el ultimo elemento
        self.s_min = math.inf
        self.s_max = -math.inf
        self.z_max = 0.0
        self.solidos: List[int] = []


class _Compilador:
    def __init__(self, doc: dict):
        self.doc = doc
        self.solidos: List[Solido] = []
        self.elementos: List[Elemento] = []
        self.inicio = (0.0, 0.0, 0.0)

    def a_odom(self, x: float, y: float) -> Tuple[float, float]:
        xi, yi, ri = self.inicio
        c, s = _cs(-ri)
        dx, dy = x - xi, y - yi
        return c * dx - s * dy, s * dx + c * dy

    def anadir(self, g: _Grupo, s0: float, t: float, rel: float, ds: float, dt: float,
               largo: float, ancho: float, zc: float, pendiente=(0.0, 0.0),
               clase='terreno', color=None, origen='') -> None:
        """Rectangulo con centro (ds, dt) en el marco del elemento, que entra en (s0, t)
        del grupo con rumbo relativo rel; techo zc en el centro con pendiente
        (dz/ds, dz/dt) en el marco del elemento."""
        ce, se = _cs(rel)
        sc = s0 + ce * ds - se * dt
        tc = t + se * ds + ce * dt
        for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            sq = sc + ce * du * largo / 2 - se * dv * ancho / 2
            g.s_min = min(g.s_min, sq)
            g.s_max = max(g.s_max, sq)
            g.fin = sq if g.fin is None else max(g.fin, sq)
        cg, sg = _cs(g.rumbo)
        xo, yo = self.a_odom(g.ox + cg * sc - sg * tc, g.oy + sg * sc + cg * tc)
        ang = _normalizar(g.rumbo + rel - self.inicio[2])
        c, s = _cs(ang)
        gs, gt = pendiente
        bx, by = c * gs - s * gt, s * gs + c * gt
        so = Solido(xo, yo, ang, largo, ancho, zc - bx * xo - by * yo, bx, by,
                    clase, color, origen)
        g.z_max = max(g.z_max, _z_max(so))
        g.solidos.append(len(self.solidos))
        self.solidos.append(so)

    # ---------------------------------------------------------- nivel superior
    def compilar(self) -> Mundo:
        d = _mapa(self.doc, 'el fichero')
        if 'formato' not in d:
            raise ErrorMundo("falta 'formato: %d'" % FORMATO)
        if d['formato'] != FORMATO or isinstance(d['formato'], bool):
            raise ErrorMundo('formato %r no soportado (solo %d)' % (d['formato'], FORMATO))
        _claves(d, CLAVES_MUNDO, 'nivel superior')
        nombre = _texto_de(d, 'nombre', 'nivel superior', '')
        descripcion = _texto_de(d, 'descripcion', 'nivel superior', '')

        ini = _mapa(d.get('inicio') or {}, 'inicio')
        _claves(ini, CLAVES_INICIO, 'inicio')
        self.inicio = (_numero(ini, 'x', 'inicio', 0.0), _numero(ini, 'y', 'inicio', 0.0),
                       math.radians(_numero(ini, 'rumbo', 'inicio', 0.0)))

        limites = None
        if 'limites' in d:
            v = d['limites']
            if not isinstance(v, list) or len(v) != 2:
                raise ErrorMundo("limites: debe ser [[x0, y0], [x1, y1]]")
            x0, y0 = _punto(v[0], 'limites', 'limites')
            x1, y1 = _punto(v[1], 'limites', 'limites')
            if x1 <= x0 or y1 <= y0:
                raise ErrorMundo('limites: la segunda esquina debe quedar arriba a la derecha')
            limites = ((x0, y0), (x1, y1))

        fichero = _Grupo('paredes', 0.0, 0.0, 0.0, None, None)
        for k, w in enumerate(_lista(d, 'paredes', 'nivel superior')):
            self.pared(_mapa(w, 'paredes, pared %d' % (k + 1)), k + 1, fichero)

        zonas = []
        for k, z in enumerate(_lista(d, 'zonas', 'nivel superior')):
            zonas.append(self.zona(_mapa(z, 'zonas, zona %d' % (k + 1)), k + 1))

        sueltos = _Grupo('elementos', 0.0, 0.0, 0.0, None, None)
        for k, e in enumerate(_lista(d, 'elementos', 'nivel superior')):
            self.elemento(e, sueltos, k + 1)

        carriles = []
        nombres = set()
        for k, c in enumerate(_lista(d, 'carriles', 'nivel superior')):
            ca = self.carril(_mapa(c, 'carril %d' % (k + 1)), k + 1)
            if ca.nombre in nombres:
                raise ErrorMundo("%s: nombre de carril repetido" % ca.nombre)
            nombres.add(ca.nombre)
            carriles.append(ca)

        return Mundo(nombre or 'sin_nombre', self.solidos, zonas, carriles, self.inicio,
                     descripcion, limites, self.elementos)

    def pared(self, w: dict, k: int, g: _Grupo) -> None:
        donde = 'paredes, pared %d' % k
        _claves(w, CLAVES_PARED, donde)
        alto = _positivo(w, 'alto', donde, ALTO_PARED)
        grosor = _positivo(w, 'grosor', donde, GROSOR_PARED)
        puntos = _lista(w, 'puntos', donde)
        if len(puntos) < 2:
            raise ErrorMundo("%s: 'puntos' necesita al menos dos [x, y]" % donde)
        pts = [_punto(p, 'puntos', donde) for p in puntos]
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            largo = math.hypot(x1 - x0, y1 - y0)
            if largo <= 0:
                raise ErrorMundo('%s: dos puntos seguidos iguales' % donde)
            self.anadir(g, (x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0),
                        0.0, 0.0, largo + grosor, grosor, alto, clase='pared',
                        origen=donde)

    def zona(self, z: dict, k: int) -> Zona:
        donde = 'zonas, zona %d' % k
        _claves(z, CLAVES_ZONA, donde)
        if 'nombre' not in z:
            raise ErrorMundo("%s: falta 'nombre'" % donde)
        nombre = _texto_de(z, 'nombre', donde, '')
        for clave in ('de', 'a'):
            if clave not in z:
                raise ErrorMundo("%s: falta '%s'" % (donde, clave))
        (xa, ya), (xb, yb) = _punto(z['de'], 'de', donde), _punto(z['a'], 'a', donde)
        x0, x1, y0, y1 = min(xa, xb), max(xa, xb), min(ya, yb), max(ya, yb)
        if x1 - x0 <= 0 or y1 - y0 <= 0:
            raise ErrorMundo('%s: la zona no tiene area' % donde)
        esquinas = tuple(self.a_odom(x, y) for x, y in
                         ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))
        return Zona(nombre, esquinas, self.a_odom((x0 + x1) / 2, (y0 + y1) / 2))

    def carril(self, c: dict, k: int) -> Carril:
        nombre = _texto_de(c, 'nombre', 'carril %d' % k, 'carril_%d' % k)
        donde = nombre
        _claves(c, CLAVES_CARRIL, donde)
        titulo = _texto_de(c, 'titulo', donde, nombre)
        if 'origen' not in c:
            raise ErrorMundo("%s: falta 'origen'" % donde)
        ox, oy = _punto(c['origen'], 'origen', donde)
        rumbo = math.radians(_numero(c, 'rumbo', donde, 0.0))
        largo = _positivo(c, 'largo', donde) if 'largo' in c else None
        ancho = _positivo(c, 'ancho', donde) if 'ancho' in c else None
        color = _color(c, donde)
        g = _Grupo(nombre, ox, oy, rumbo, ancho, color)
        for n, e in enumerate(_lista(c, 'elementos', donde)):
            self.elemento(e, g, n + 1)
        xo, yo = self.a_odom(ox, oy)
        vacio = not g.solidos
        return Carril(nombre, titulo, (xo, yo), _normalizar(rumbo - self.inicio[2]),
                      largo, ancho, color, 0.0 if vacio else g.s_min,
                      0.0 if vacio else g.s_max, g.z_max, tuple(g.solidos))

    # ---------------------------------------------------------- elementos
    def elemento(self, e, g: _Grupo, k: int) -> None:
        tipo = e.get('tipo') if isinstance(e, dict) else None
        donde = '%s, elemento %d (%s)' % (g.nombre, k, tipo if tipo is not None else '?')
        e = _mapa(e, donde)
        if tipo is None:
            raise ErrorMundo("%s: falta 'tipo'" % donde)
        if not isinstance(tipo, str):
            raise ErrorMundo("%s: 'tipo' debe ser un texto" % donde)
        if tipo in TIPOS_FUERA:
            raise ErrorMundo('%s: tipo %s no soportado en formato 1' % (donde, tipo))
        if tipo not in CLAVES_TIPO:
            raise ErrorMundo("%s: tipo desconocido (%s)" % (donde, ', '.join(CLAVES_TIPO)))
        _claves(e, ('tipo', 'color') + CLAVES_TIPO[tipo], donde, tipo)
        color = _color(e, donde) or g.color
        antes = len(self.solidos)

        s0 = t = 0.0
        if tipo != 'viga':
            if 'pos' not in e:
                raise ErrorMundo("%s: falta 'pos'" % donde)
            pos = e['pos']
            if not isinstance(pos, list) or len(pos) != 2:
                raise ErrorMundo("%s: 'pos' debe ser [s, t] o [sigue, t]" % donde)
            if pos[0] == 'sigue':
                if g.fin is None:
                    raise ErrorMundo("%s: 'sigue' sin elemento anterior" % donde)
                s0 = g.fin
            else:
                s0 = _valor(pos[0], 'pos', donde)
            t = _valor(pos[1], 'pos', donde)
        g.fin = None
        matriz = None
        cortado = 0.0

        def ancho_de() -> float:
            if 'ancho' in e:
                return _positivo(e, 'ancho', donde)
            if g.ancho is None:
                raise ErrorMundo("%s: falta 'ancho' (ni en el elemento ni en el carril)"
                                 % donde)
            return g.ancho

        rel = math.radians(_numero(e, 'rumbo', donde, 0.0))
        if tipo == 'caja':
            largo = _positivo(e, 'largo', donde)
            alto = _positivo(e, 'alto', donde)
            self.anadir(g, s0, t, rel, largo / 2, 0.0, largo, ancho_de(), alto,
                        color=color, origen=donde)

        elif tipo == 'rampa':
            ancho = ancho_de()
            z = _cota(e, 'z', donde)
            tiene = _dos_de(e, ('largo', 'alto', 'pendiente'), 2, donde, 'una rampa')
            if 'pendiente' in tiene:
                p = _numero(e, 'pendiente', donde)
                if not -90 < p < 90 or p == 0:
                    raise ErrorMundo("%s: 'pendiente' debe estar entre -90 y 90 grados "
                                     "y no ser 0 (es %g)" % (donde, p))
            if 'largo' in tiene:
                largo = _positivo(e, 'largo', donde)
                alto = (_numero(e, 'alto', donde) if 'alto' in tiene
                        else largo * math.tan(math.radians(p)))
            else:
                alto = _numero(e, 'alto', donde)
                if alto == 0:
                    raise ErrorMundo("%s: 'alto' no puede ser 0 en una rampa" % donde)
                largo = abs(alto) / math.tan(math.radians(abs(p)))
            if alto == 0:
                raise ErrorMundo('%s: una rampa sin desnivel es una caja' % donde)
            if z + alto < -1e-12:
                raise ErrorMundo('%s: la rampa acaba por debajo del suelo (z + alto = %g)'
                                 % (donde, z + alto))
            self.anadir(g, s0, t, rel, largo / 2, 0.0, largo, ancho, z + alto / 2,
                        (alto / largo, 0.0), color=color, origen=donde)

        elif tipo == 'escalera':
            ancho = ancho_de()
            z = _cota(e, 'z', donde)
            n = _entero(e, 'peldanos', donde)
            h = _positivo(e, 'contrahuella', donde)
            _dos_de(e, ('huella', 'pendiente'), 1, donde, 'una escalera')
            huella = (_positivo(e, 'huella', donde) if 'huella' in e
                      else h / math.tan(_angulo_agudo(e, 'pendiente', donde)))
            rellano = _positivo(e, 'rellano', donde, huella)
            s = 0.0
            for i in range(1, n):
                self.anadir(g, s0, t, rel, s + huella / 2, 0.0, huella, ancho, z + i * h,
                            color=color, origen=donde)
                s += huella
            self.anadir(g, s0, t, rel, s + rellano / 2, 0.0, rellano, ancho, z + n * h,
                        color=color, origen=donde)
            s += rellano
            if 'bajada' in e:
                b = _mapa(e['bajada'], donde + ', bajada')
                _claves(b, ('huella', 'pendiente'), donde + ', bajada')
                _dos_de(b, ('huella', 'pendiente'), 1, donde + ', bajada', 'la bajada')
                hb = (_positivo(b, 'huella', donde + ', bajada') if 'huella' in b
                      else h / math.tan(_angulo_agudo(b, 'pendiente', donde + ', bajada')))
                for i in range(n - 1, 0, -1):
                    self.anadir(g, s0, t, rel, s + hb / 2, 0.0, hb, ancho, z + i * h,
                                color=color, origen=donde)
                    s += hb
            else:
                cortado = z + n * h

        elif tipo == 'campo_escalones':
            celda = _positivo(e, 'celda', donde, 0.10)
            unidad = _positivo(e, 'unidad', donde, 0.10)
            base = _cota(e, 'base', donde)
            _dos_de(e, ('alturas', 'patron'), 1, donde, 'un campo de escalones')
            if 'alturas' in e:
                for clave in ('filas', 'columnas'):
                    if clave in e:
                        raise ErrorMundo("%s: '%s' solo va con 'patron'" % (donde, clave))
                m = _matriz(e['alturas'], 'alturas', donde)
                for fila in m:
                    for a in fila:
                        if isinstance(a, bool) or not isinstance(a, int) or a < 0:
                            raise ErrorMundo("%s: 'alturas' lleva enteros >= 0 (hay %r)"
                                             % (donde, a))
            else:
                m = patron_escalones(e['patron'], _entero(e, 'filas', donde),
                                     _entero(e, 'columnas', donde), donde)
            ancho_campo = len(m[0]) * celda
            for i, fila in enumerate(m):
                for j, a in enumerate(fila):
                    if a == 0:
                        continue
                    self.anadir(g, s0, t, 0.0, (i + 0.5) * celda,
                                ancho_campo / 2 - (j + 0.5) * celda, celda, celda,
                                base + unidad * a, color=color, origen=donde)
            matriz = tuple(tuple(f) for f in m)
            g.fin = max(s0 + len(m) * celda, -math.inf if g.fin is None else g.fin)

        elif tipo == 'campo_rampas':
            celda = _positivo(e, 'celda', donde, 0.6)
            p = _angulo_agudo(e, 'pendiente', donde, 15.0)
            _dos_de(e, ('orientaciones', 'patron'), 1, donde, 'un campo de rampas')
            if 'orientaciones' in e:
                for clave in ('filas', 'columnas'):
                    if clave in e:
                        raise ErrorMundo("%s: '%s' solo va con 'patron'" % (donde, clave))
                m = _matriz(e['orientaciones'], 'orientaciones', donde)
                for fila in m:
                    for o in fila:
                        if not isinstance(o, str) or o not in SUBIDA:
                            raise ErrorMundo("%s: orientacion %r (A, T, I o D)" % (donde, o))
            else:
                m = patron_rampas(e['patron'], _entero(e, 'filas', donde),
                                  _entero(e, 'columnas', donde), donde)
            tg = math.tan(p)
            alto = celda * tg
            ancho_campo = len(m[0]) * celda
            for i, fila in enumerate(m):
                for j, o in enumerate(fila):
                    us, ut = SUBIDA[o]
                    self.anadir(g, s0, t, 0.0, (i + 0.5) * celda,
                                ancho_campo / 2 - (j + 0.5) * celda, celda, celda,
                                alto / 2, (us * tg, ut * tg), color=color, origen=donde)

        elif tipo == 'viga':
            for clave in ('de', 'a'):
                if clave not in e:
                    raise ErrorMundo("%s: falta '%s'" % (donde, clave))
            sa, ta = _punto(e['de'], 'de', donde)
            sb, tb = _punto(e['a'], 'a', donde)
            seccion = _positivo(e, 'seccion', donde, 0.10)
            alto = _positivo(e, 'alto', donde, seccion)
            largo = math.hypot(sb - sa, tb - ta)
            if largo <= 0:
                raise ErrorMundo("%s: 'de' y 'a' coinciden" % donde)
            self.anadir(g, (sa + sb) / 2, (ta + tb) / 2, math.atan2(tb - ta, sb - sa),
                        0.0, 0.0, largo, seccion, alto, color=color, origen=donde)

        if g.fin is None:
            g.fin = s0
        self.elementos.append(Elemento(g.nombre, k, tipo,
                                       tuple(range(antes, len(self.solidos))),
                                       matriz, cortado))


class _CargadorYaml(yaml.SafeLoader):
    """SafeLoader que no deja pasar claves repetidas."""


def _mapa_sin_repetidas(cargador, nodo, deep=False):
    cargador.flatten_mapping(nodo)
    vistas = set()
    for nodo_clave, _ in nodo.value:
        clave = cargador.construct_object(nodo_clave, deep=deep)
        try:
            repetida = clave in vistas
        except TypeError:
            continue
        if repetida:
            raise ErrorMundo("linea %d: clave repetida '%s'"
                             % (nodo_clave.start_mark.line + 1, clave))
        vistas.add(clave)
    return cargador.construct_mapping(nodo, deep=deep)


_CargadorYaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                              _mapa_sin_repetidas)


def cargar(texto_yaml: str, nombre_fichero: str = '') -> Mundo:
    """Del texto YAML al mundo, o ErrorMundo con un mensaje que dice donde."""
    prefijo = nombre_fichero + ': ' if nombre_fichero else ''
    try:
        doc = yaml.load(texto_yaml, Loader=_CargadorYaml)
    except ErrorMundo as e:
        raise ErrorMundo(prefijo + str(e)) from None
    except yaml.YAMLError as e:
        raise ErrorMundo(prefijo + 'YAML mal formado: ' + ' '.join(str(e).split())) from None
    try:
        mundo = _Compilador(doc).compilar()
    except ErrorMundo as e:
        raise ErrorMundo(prefijo + str(e)) from None
    if mundo.nombre == 'sin_nombre' and nombre_fichero:
        mundo.nombre = os.path.splitext(os.path.basename(nombre_fichero))[0]
    return mundo


def cargar_fichero(ruta: str) -> Mundo:
    try:
        with open(ruta, encoding='utf-8') as f:
            texto = f.read()
    except (OSError, UnicodeDecodeError) as e:
        raise ErrorMundo('%s: no se puede leer (%s)' % (ruta, e)) from None
    return cargar(texto, ruta)


def resolver_ruta(nombre_o_ruta: str) -> str:
    """'escalon' -> share/starcrawler_sim/mundos/escalon.yaml; una ruta, tal cual."""
    if nombre_o_ruta.endswith('.yaml') or '/' in nombre_o_ruta or os.sep in nombre_o_ruta:
        return nombre_o_ruta
    fichero = nombre_o_ruta + '.yaml'
    buscadas = []
    try:
        from ament_index_python.packages import get_package_share_directory
        buscadas.append(os.path.join(get_package_share_directory('starcrawler_sim'),
                                     'mundos', fichero))
    except Exception:
        pass
    # Desde el arbol de fuentes (tests y mundo_check sin instalar)
    buscadas.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 os.pardir, 'mundos', fichero))
    for ruta in buscadas:
        if os.path.isfile(ruta):
            return os.path.normpath(ruta)
    raise ErrorMundo("no hay mundo '%s' (buscado en %s)"
                     % (nombre_o_ruta, ', '.join(os.path.normpath(r) for r in buscadas)))
