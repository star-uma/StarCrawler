"""
contacto_core.py — el robot apoyado en un terreno con obstaculos, sin ROS
=========================================================================
Modulo PURO, como chasis_core.py, cuya regla de las diagonales generaliza.
Modelo cuasiestatico, sin dinamica:

- el terreno se mira en 9 cortes verticales paralelos al rumbo (2 por lado
  de orugas y 5 en el chasis); en cada uno es un perfil (s, z);
- cada pieza (oruga: envolvente de sus dos poleas; chasis: rectangulo) da
  RESTRICCIONES: la z de base_link minima para no atravesar un tramo o un
  vertice del perfil, con su punto y su normal. Segun el angulo de la normal
  con la vertical son APOYO (sube el chasis) o PARED (empuja la base a lo
  largo del rumbo: es el bloqueo);
- el reposo es el plano de apoyo con brazos de palanca; si el CdG se sale
  del poligono de apoyo, vuelca sobre la arista hasta el primer contacto.

Solo observa: nunca frena brazos ni traccion. Diseno y numeros en
docs/diseno_arena/especificacion.md (§5). Convenio REP-103, como
chasis_core: cabeceo + = morro abajo, balanceo + = lado izquierdo arriba.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from bisect import bisect_left, bisect_right
from collections import namedtuple
from dataclasses import dataclass, field, replace
from typing import List, Optional, Protocol, Sequence, Tuple

from starcrawler_odometry.chasis_core import (JUNTAS_ORUGA, Geometria,
                                              geometria_desde_urdf)
from starcrawler_odometry.odometry_core import integrar

PIEZAS = ('FR', 'FL', 'RR', 'RL', 'chasis')
CHASIS = 4

_INF = math.inf
_S0, _S1 = -1.0, 1.0                    # ventana del perfil de cada linea
# Un perfil vale desplazado solo si el rumbo no cambia: con la pared casi
# paralela a la linea, 0,05 grados mueven su entrada varios cm (§5.3 decia 0,05)
_CACHE_YAW = 1e-9
_CACHE_LATERAL = 0.0005
_CACHE_AVANCE = 0.15
_HOLGURA_GUARDIA = 0.002
_BISECCIONES_VUELCO = 10
_BISECCIONES_GUARDIA = 6
# colocar(): sin tope por tick
_ITER_COLOCAR = 50
_GIRO_COLOCAR = math.radians(45.0)      # correccion maxima del plano por iteracion
_VUELCO_COLOCAR = math.radians(20.0)    # inclinacion maxima de cada vuelco
_BISECCIONES_COLOCAR = 20
_QUIETO_PASO = 1e-6                     # rad: el reposo deja de iterar
_QUIETO_COLOCAR = 1e-12
_MAX_CONTACTOS = 24                     # de apoyo, en Estado.contactos
_EXTRA_EMPUJE = 1e-6


class Terreno(Protocol):
    def perfil(self, x0: float, y0: float, ux: float, uy: float,
               s0: float, s1: float) -> Sequence[Tuple[float, float]]: ...


@dataclass(frozen=True)
class GeometriaRobot:
    orugas: Geometria
    chasis: Tuple[float, float, float]      # largo, ancho, alto
    ancho_oruga: float
    masa_chasis: float
    masa_brazo: float


@dataclass(frozen=True)
class Parametros:
    ataque_banda: float = math.radians(75.0)
    ataque_casco: float = math.radians(60.0)
    traccion_lisa: float = math.radians(40.0)
    caida_ang: float = math.radians(90.0)   # rad/s
    caida_lin: float = 1.0                  # m/s
    vuelco: float = math.radians(75.0)
    subpaso: float = 0.002
    empuje_max: float = 0.005
    tol_apoyo: float = 0.0005
    tol_traccion: float = 0.002
    tol_penetra: float = 0.0001
    eps_equilibrio: float = 0.001
    lineas_oruga: Tuple[float, ...] = (-0.035, 0.035)
    lineas_chasis: Tuple[float, ...] = (-0.19, -0.095, 0.0, 0.095, 0.19)
    iteraciones: int = 8
    max_giro_iter: float = math.radians(5.0)
    relajacion: float = 0.5
    ventana_candidatos: float = 0.06
    max_candidatos: int = 10


@dataclass(frozen=True)
class Pose3D:
    x: float            # base_footprint en el mundo
    y: float
    yaw: float
    z: float            # base_link (los pivotes) sobre z = 0
    cabeceo: float
    balanceo: float


@dataclass(frozen=True)
class Contacto:
    pieza: str
    x: float
    y: float
    z: float
    normal: Tuple[float, float, float]      # terreno -> robot, en el mundo
    ataque: float                           # rad, de la normal con la vertical
    tipo: str                               # 'apoyo' | 'pared'
    traccion: bool
    holgura: float                          # z de la pose - la requerida


@dataclass(frozen=True)
class Estado:
    pose: Pose3D
    elevaciones: Tuple[float, ...]
    contactos: Tuple[Contacto, ...]
    apoya: Tuple[bool, ...]
    holguras: Tuple[float, ...]
    panza: bool
    bloqueado: Optional[str]
    sin_traccion: bool
    cayendo: bool
    volcado: bool
    atrapado: bool
    cdg: Tuple[float, float, float]
    poligono: Tuple[Tuple[float, float, float], ...]   # antihorario en planta
    margen: float                           # del CdG al borde; + = dentro
    avance: float                           # m a lo largo del rumbo, este tick
    propuesto: float                        # lo que proponian las orugas
    soporte: tuple = ()
    cache: dict = field(default_factory=dict, compare=False, repr=False)


# Restriccion de una pieza en una linea, como tupla por rendimiento
_H, _P, _S, _L, _ZC, _NS, _NZ, _AP, _TR, _EM, _LI, _K = range(12)
Restriccion = namedtuple('Restriccion', 'altura pieza s l z ns nz apoyo '
                                        'traccion empuje linea clave')


# ================================================================ URDF

def _masa(link, nombre: str) -> float:
    m = link.find('inertial/mass')
    if m is None or m.get('value') is None:
        raise ValueError('%s sin <inertial><mass>' % nombre)
    return float(m.get('value'))


def geometria_robot_desde_urdf(xml: str) -> GeometriaRobot:
    """chasis_core.geometria_desde_urdf mas la caja del chasis, el ancho de
    las orugas y las masas."""
    orugas = geometria_desde_urdf(xml)
    robot = ET.fromstring(xml)
    juntas = {j.get('name'): j for j in robot.findall('joint')}
    links = {lk.get('name'): lk for lk in robot.findall('link')}
    base = links.get('base_link')
    if base is None:
        raise ValueError('Falta base_link en el URDF')
    caja = None
    for vis in base.findall('visual'):
        b = vis.find('geometry/box')
        if b is not None:
            caja = tuple(float(v) for v in b.get('size').split())
            break
    if caja is None or len(caja) != 3:
        raise ValueError('base_link necesita una caja visual (el chasis)')
    anchos, masas = set(), set()
    for nombre in JUNTAS_ORUGA:
        hijo = juntas[nombre].find('child').get('link')
        for vis in links[hijo].findall('visual'):
            cil = vis.find('geometry/cylinder')
            if cil is not None and cil.get('length') is not None:
                anchos.add(round(float(cil.get('length')), 9))
        masas.add(round(_masa(links[hijo], hijo), 9))
    if len(anchos) != 1 or len(masas) != 1:
        raise ValueError('Las cuatro orugas deben tener el mismo ancho y masa')
    return GeometriaRobot(orugas, caja, anchos.pop(), _masa(base, 'base_link'),
                          masas.pop())


def altura_trepable(q: float, geo: GeometriaRobot, par: Parametros = Parametros()) -> float:
    """Canto mas alto que sube el primer contacto con el chasis en llano (§5.6)."""
    o = geo.orugas
    beta = math.asin((o.radio_polea - o.radio_punta) / o.largo)
    phi = par.ataque_banda
    if q <= -beta:                          # de pie sobre las puntas
        return o.radio_punta * (1.0 - math.cos(phi))
    if q + beta <= phi:
        return o.radio_polea + o.largo * math.sin(q) - o.radio_punta * math.cos(phi)
    return o.radio_polea * (1.0 - math.cos(phi))


# ================================================================ formas

def _forma_oruga(px: float, q: float, o: Geometria, cth: float, sth: float):
    """Poleas y tangentes de abajo de una oruga, en el plano de la linea."""
    sig = 1.0 if px > 0.0 else -1.0
    r1, r2, L = o.radio_polea, o.radio_punta, o.largo
    bx2, bz2 = px + sig * L * math.cos(q), L * math.sin(q)
    c1s, c1z = cth * px, -sth * px
    c2s, c2z = cth * bx2 + sth * bz2, -sth * bx2 + cth * bz2
    dx, dz = c2s - c1s, c2z - c1z
    a = math.atan2(dz, dx)
    gam = math.acos(max(-1.0, min(1.0, (r1 - r2) / math.hypot(dx, dz))))
    tang = []
    for gg in (a + gam, a - gam):
        nx, nz = math.cos(gg), math.sin(gg)     # normal exterior
        if nz < -1e-12:
            ps, pz = c1s + r1 * nx, c1z + r1 * nz
            qs, qz = c2s + r2 * nx, c2z + r2 * nz
            if qs < ps:
                ps, pz, qs, qz = qs, qz, ps, pz
            if qs - ps > 1e-12:
                tang.append((ps, pz, qs, (qz - pz) / (qs - ps), -nx, -nz))
    return (c1s, c1z, r1, c2s, c2z, r2, tuple(tang),
            min(c1s - r1, c2s - r2), max(c1s + r1, c2s + r2))


def _forma_chasis(largo: float, alto: float, cth: float, sth: float):
    """Esquinas y aristas de abajo del chasis, en el plano de la linea."""
    a, c = largo / 2.0, alto / 2.0
    esq = [(cth * bx + sth * bz, -sth * bx + cth * bz)
           for bx, bz in ((-a, -c), (a, -c), (a, c), (-a, c))]  # antihorario
    bordes, bajas = [], set()
    for i in range(4):
        (ps, pz), (qs, qz) = esq[i], esq[(i + 1) % 4]
        if qs - ps > 1e-12:                     # avanza hacia +s: es de abajo
            n = math.hypot(qs - ps, qz - pz)
            bordes.append((ps, pz, qs, (qz - pz) / (qs - ps),
                           -(qz - pz) / n, (qs - ps) / n))
            bajas.update((i, (i + 1) % 4))
    ss = [p[0] for p in esq]
    return (tuple(esq[i] for i in sorted(bajas)), tuple(bordes), min(ss), max(ss))


def forma_oruga(i: int, q: float, geo: GeometriaRobot, cabeceo: float = 0.0):
    """La oruga i en el plano x-z del cuerpo girado por el cabeceo (tests)."""
    px = geo.orugas.pivotes[i][0]
    return (i, _forma_oruga(px, q, geo.orugas, math.cos(cabeceo), math.sin(cabeceo)))


def forma_chasis(geo: GeometriaRobot, cabeceo: float = 0.0):
    return (CHASIS, _forma_chasis(geo.chasis[0], geo.chasis[2],
                                  math.cos(cabeceo), math.sin(cabeceo)))


def punto_soporte(forma, nx: float, nz: float):
    """(s, z, polea, valor) del punto de la oruga mas lejano en la direccion n."""
    f = forma[1]
    v1 = f[0] * nx + f[1] * nz + f[2]
    v2 = f[3] * nx + f[4] * nz + f[5]
    if v1 >= v2:
        return f[0] + f[2] * nx, f[1] + f[2] * nz, 1, v1
    return f[3] + f[5] * nx, f[4] + f[5] * nz, 2, v2


def borde_inferior(forma, s: float) -> float:
    """z del borde de abajo de la oruga en s (inf fuera de su alcance)."""
    c1s, c1z, r1, c2s, c2z, r2, tang, _, _ = forma[1]
    b = _INF
    for cs, cz, r in ((c1s, c1z, r1), (c2s, c2z, r2)):
        d = s - cs
        if -r <= d <= r:
            b = min(b, cz - math.sqrt(max(0.0, r * r - d * d)))
    for ps, pz, qs, m, _, _ in tang:
        if ps <= s <= qs:
            b = min(b, pz + m * (s - ps))
    return b


# ================================================================ perfiles

def _preparar(pts, par: Parametros):
    """Del perfil (s, z) a vertices, tramos y traccion de cada rasgo."""
    vs = [float(p[0]) for p in pts]
    vz = [float(p[1]) for p in pts]
    n = len(vs)
    tlisa = math.tan(par.traccion_lisa)
    tramos = []
    for i in range(n - 1):
        ds = vs[i + 1] - vs[i]
        if ds > 1e-12:
            m = (vz[i + 1] - vz[i]) / ds
            k = 1.0 / math.sqrt(1.0 + m * m)
            tramos.append((vs[i], vs[i + 1], vz[i], m, -m * k, k, abs(m) <= tlisa))
    vtrac = []
    for i in range(n):
        canto = False
        if 0 < i < n - 1:
            d1s, d1z = vs[i] - vs[i - 1], vz[i] - vz[i - 1]
            d2s, d2z = vs[i + 1] - vs[i], vz[i + 1] - vz[i]
            canto = d1s * d2z - d1z * d2s < -1e-12
        suave = True
        for a in (i - 1, i):
            if 0 <= a < n - 1:
                ds = vs[a + 1] - vs[a]
                if ds > 1e-12 and abs(vz[a + 1] - vz[a]) > tlisa * ds:
                    suave = False
        vtrac.append(canto or suave)
    return vs, vz, vtrac, tramos, [t[1] for t in tramos]


# ================================================================ restricciones

def _restr_oruga(pf, f, p, sh, dz, off, lat, z, cosb, li, out) -> float:
    """Restricciones de una oruga en una linea; devuelve su H maxima."""
    vs, vz, vtrac, tramos, tsb = pf
    c1s, c1z, r1, c2s, c2z, r2, tang, smin, smax = f
    c1s += sh
    c2s += sh
    c1z += dz
    c2z += dz
    smin += sh
    smax += sh
    tg = [(ps + sh, pz + dz, qs + sh, m, ins, inz) for ps, pz, qs, m, ins, inz in tang]
    hmax = -_INF
    for j in range(bisect_left(vs, smin), bisect_right(vs, smax)):
        s = vs[j]
        zv = vz[j]
        b = _INF
        cual = 0
        d = s - c1s
        if -r1 < d < r1:
            b = c1z - math.sqrt(r1 * r1 - d * d)
            cual = 1
        d = s - c2s
        if -r2 < d < r2:
            bb = c2z - math.sqrt(r2 * r2 - d * d)
            if bb < b:
                b = bb
                cual = 2
        seg = None
        for t in tg:
            if t[0] <= s <= t[2]:
                bb = t[1] + t[3] * (s - t[0])
                if bb < b:
                    b = bb
                    seg = t
        if b == _INF:
            continue
        H = zv - b
        if seg is not None:
            ns, nz = seg[4], seg[5]
            emp = (H - z) * nz / ns if abs(ns) > 1e-9 else 0.0
        else:
            cs, cz, r = (c1s, c1z, r1) if cual == 1 else (c2s, c2z, r2)
            ns = cs - s
            nz = z + cz - zv                    # hacia el centro, en la pose actual
            n = math.hypot(ns, nz)
            if n < 1e-12:
                ns, nz = 0.0, 1.0
            else:
                ns /= n
                nz /= n
            dv = max(0.0, z + cz - zv)
            falta = math.sqrt(max(0.0, r * r - dv * dv)) - abs(cs - s)
            emp = math.copysign(max(0.0, falta), cs - s if cs != s else -1.0)
        out.append((H, p, s - off, lat, zv, ns, nz, nz >= cosb, vtrac[j], emp, li,
                    (li, p, 'v', j)))
        if H > hmax:
            hmax = H
    nt = len(tramos)
    k = bisect_left(tsb, smin)
    while k < nt:
        sa, sb, za, m, ns, nz, tr = tramos[k]
        if sa > smax:
            break
        for cs, cz, r, ci in ((c1s, c1z, r1, 1), (c2s, c2z, r2, 2)):
            sf = cs - r * ns
            if sa - 1e-12 <= sf <= sb + 1e-12:
                H = za + m * (cs - sa) + r / nz - cz
                emp = (H - z) * nz / ns if abs(ns) > 1e-9 else 0.0
                out.append((H, p, sf - off, lat, za + m * (sf - sa), ns, nz, nz >= cosb,
                            tr, emp, li, (li, p, 't', k, ci)))
                if H > hmax:
                    hmax = H
        k += 1
    return hmax


def _restr_chasis(pf, f, sh, dz, off, lat, z, cosc, li, out) -> float:
    vs, vz, _, tramos, tsb = pf
    esq, bordes, smin, smax = f
    smin += sh
    smax += sh
    bd = [(ps + sh, pz + dz, qs + sh, m, ins, inz) for ps, pz, qs, m, ins, inz in bordes]
    hmax = -_INF
    for j in range(bisect_left(vs, smin), bisect_right(vs, smax)):
        s = vs[j]
        b = _INF
        seg = None
        for t in bd:
            if t[0] <= s <= t[2]:
                bb = t[1] + t[3] * (s - t[0])
                if bb < b:
                    b = bb
                    seg = t
        if seg is None:
            continue
        H = vz[j] - b
        ns, nz = seg[4], seg[5]
        emp = (H - z) * nz / ns if abs(ns) > 1e-9 else 0.0
        out.append((H, CHASIS, s - off, lat, vz[j], ns, nz, nz >= cosc, False, emp, li,
                    (li, CHASIS, 'v', j)))
        if H > hmax:
            hmax = H
    nt = len(tramos)
    k = bisect_left(tsb, smin)
    while k < nt:
        sa, sb, za, m, ns, nz, _ = tramos[k]
        if sa > smax:
            break
        for e, (ks, kz) in enumerate(esq):
            ks += sh
            if sa - 1e-12 <= ks <= sb + 1e-12:
                zt = za + m * (ks - sa)
                H = zt - kz - dz
                emp = (H - z) * nz / ns if abs(ns) > 1e-9 else 0.0
                out.append((H, CHASIS, ks - off, lat, zt, ns, nz, nz >= cosc, False, emp,
                            li, (li, CHASIS, 't', k, e)))
                if H > hmax:
                    hmax = H
        k += 1
    return hmax


def restricciones_linea(perfil, formas, z: float, par: Parametros = Parametros(),
                        lat: float = 0.0) -> List[Restriccion]:
    """Restricciones de unas formas (forma_oruga / forma_chasis) sobre un perfil."""
    pf = _preparar(perfil, par)
    out = []
    cosb, cosc = math.cos(par.ataque_banda), math.cos(par.ataque_casco)
    for p, f in formas:
        if p == CHASIS:
            _restr_chasis(pf, f, 0.0, 0.0, 0.0, lat, z, cosc, 0, out)
        else:
            _restr_oruga(pf, f, p, 0.0, 0.0, 0.0, lat, z, cosb, 0, out)
    return [Restriccion(*c) for c in out]


# ================================================================ evaluacion

class _Ev:
    """Restricciones de todas las lineas en una pose."""
    __slots__ = ('cons', 'zreq', 'hp')

    def __init__(self, cons, zreq, hp):
        self.cons = cons
        self.zreq = zreq
        self.hp = hp


class _Mov:
    """Pose de trabajo: la de base_footprint y la de base_link."""
    __slots__ = ('x', 'y', 'yaw', 'z', 'th', 'ph')

    def __init__(self, x, y, yaw, z, th, ph):
        self.x, self.y, self.yaw, self.z, self.th, self.ph = x, y, yaw, z, th, ph

    def copia(self):
        return _Mov(self.x, self.y, self.yaw, self.z, self.th, self.ph)

    def poner(self, o):
        self.x, self.y, self.yaw, self.z, self.th, self.ph = o.x, o.y, o.yaw, o.z, o.th, o.ph


class _Ctx:
    def __init__(self, terreno, geo: GeometriaRobot, par: Parametros, perfiles: dict):
        self.ter = terreno
        self.geo = geo
        self.par = par
        self.perfiles = perfiles
        self.cosb = math.cos(par.ataque_banda)
        self.cosc = math.cos(par.ataque_casco)
        o = geo.orugas
        self.pivotes = [(float(x), float(y)) for x, y in o.pivotes]
        lineas = []
        for y in sorted({round(y, 9) for _, y in self.pivotes}):
            piezas = tuple(i for i, (_, yi) in enumerate(self.pivotes)
                           if round(yi, 9) == y)
            for dl in par.lineas_oruga:
                lineas.append((y + dl, piezas))
        for lat in par.lineas_chasis:
            lineas.append((lat, (CHASIS,)))
        self.lineas = lineas
        # Distancia maxima de cada pieza a la base, para la guardia
        extremo = o.largo + o.radio_punta
        self.radio = [math.hypot(abs(x) + extremo, abs(y) + geo.ancho_oruga / 2.0)
                      for x, y in self.pivotes]
        self.radio.append(math.hypot(geo.chasis[0] / 2.0, geo.chasis[1] / 2.0))

    def perfil(self, li, bx, by, yaw, cy, sy):
        e = self.perfiles.get(li)
        if e is not None:
            bx0, by0, yaw0, cy0, sy0, pf = e
            dx, dy = bx - bx0, by - by0
            off = dx * cy0 + dy * sy0
            dyaw = (yaw - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
            if (-_CACHE_AVANCE <= off <= _CACHE_AVANCE
                    and abs(dy * cy0 - dx * sy0) <= _CACHE_LATERAL
                    and abs(dyaw) <= _CACHE_YAW):
                return pf, off
        pf = _preparar(self.ter.perfil(bx, by, cy, sy, _S0, _S1), self.par)
        self.perfiles[li] = (bx, by, yaw, cy, sy, pf)
        return pf, 0.0

    def evaluar(self, m: _Mov, q) -> _Ev:
        o = self.geo.orugas
        cth, sth = math.cos(m.th), math.sin(m.th)
        sph = math.sin(m.ph)
        formas = [_forma_oruga(px, q[i], o, cth, sth)
                  for i, (px, _) in enumerate(self.pivotes)]
        fch = _forma_chasis(self.geo.chasis[0], self.geo.chasis[2], cth, sth)
        cy, sy = math.cos(m.yaw), math.sin(m.yaw)
        z = m.z
        cons = []
        hp = {}
        for li, (lat, piezas) in enumerate(self.lineas):
            pf, off = self.perfil(li, m.x - lat * sy, m.y + lat * cy, m.yaw, cy, sy)
            sh = sth * sph * lat + off
            dz = cth * sph * lat
            for p in piezas:
                if p == CHASIS:
                    hp[(li, p)] = _restr_chasis(pf, fch, sh, dz, off, lat, z,
                                                self.cosc, li, cons)
                else:
                    hp[(li, p)] = _restr_oruga(pf, formas[p], p, sh, dz, off, lat, z,
                                               self.cosb, li, cons)
        zreq = -_INF
        for c in cons:
            if c[_AP] and c[_H] > zreq:
                zreq = c[_H]
        return _Ev(cons, zreq, hp)

    # --- CdG ---------------------------------------------------------------
    def cdg_cuerpo(self, q):
        o = self.geo.orugas
        mb = self.geo.masa_brazo
        total = self.geo.masa_chasis + mb * len(self.pivotes)
        cx = cy = cz = 0.0
        for (px, py), qi in zip(self.pivotes, q):
            sig = 1.0 if px > 0.0 else -1.0
            cx += mb * (px + sig * o.largo / 2.0 * math.cos(qi))
            cy += mb * py
            cz += mb * o.largo / 2.0 * math.sin(qi)
        return cx / total, cy / total, cz / total

    def cdg(self, m: _Mov, q):
        """(s, l) en planta, en el marco del rumbo, y z en el mundo."""
        cx, cy, cz = self.cdg_cuerpo(q)
        cth, sth = math.cos(m.th), math.sin(m.th)
        cph, sph = math.cos(m.ph), math.sin(m.ph)
        vz = cy * sph + cz * cph
        return (cth * cx + sth * vz, cy * cph - cz * sph,
                m.z - sth * cx + cth * vz)

    def mundo(self, m: _Mov, s: float, lat: float) -> Tuple[float, float]:
        cy, sy = math.cos(m.yaw), math.sin(m.yaw)
        return m.x + s * cy - lat * sy, m.y + s * sy + lat * cy


# ================================================================ planta

def _envolvente(pts) -> List[int]:
    """Indices de la envolvente convexa, antihoraria (monotone chain)."""
    vistos = {}
    for i, (s, lat) in enumerate(pts):
        vistos.setdefault((round(s, 9), round(lat, 9)), i)
    idx = sorted(vistos.values(), key=lambda i: pts[i])
    if len(idx) <= 2:
        return idx

    def cruz(o, a, b):
        return ((pts[a][0] - pts[o][0]) * (pts[b][1] - pts[o][1])
                - (pts[a][1] - pts[o][1]) * (pts[b][0] - pts[o][0]))

    lo, up = [], []
    for i in idx:
        while len(lo) >= 2 and cruz(lo[-2], lo[-1], i) <= 1e-14:
            lo.pop()
        lo.append(i)
    for i in reversed(idx):
        while len(up) >= 2 and cruz(up[-2], up[-1], i) <= 1e-14:
            up.pop()
        up.append(i)
    casco = lo[:-1] + up[:-1]
    return casco if len(casco) >= 2 else [idx[0], idx[-1]]


def _a_segmento(g, a, b):
    """(distancia, t) de g al segmento a-b."""
    es, el = b[0] - a[0], b[1] - a[1]
    n2 = es * es + el * el
    t = 0.0 if n2 < 1e-24 else max(0.0, min(1.0, ((g[0] - a[0]) * es
                                                  + (g[1] - a[1]) * el) / n2))
    return math.hypot(g[0] - a[0] - t * es, g[1] - a[1] - t * el), t


def _distancia(g, pts, casco):
    """(margen con signo, rasgo): rasgo ('a', i, j) arista o ('v', i) vertice."""
    if not casco:
        return -_INF, None
    if len(casco) == 1:
        i = casco[0]
        return -math.hypot(g[0] - pts[i][0], g[1] - pts[i][1]), ('v', i)
    n = len(casco)
    if n >= 3:
        peor, arista = _INF, None
        for k in range(n):
            a, b = pts[casco[k]], pts[casco[(k + 1) % n]]
            es, el = b[0] - a[0], b[1] - a[1]
            d = (es * (g[1] - a[1]) - el * (g[0] - a[0])) / math.hypot(es, el)
            if d < peor:
                peor, arista = d, (casco[k], casco[(k + 1) % n])
        if peor >= 0.0:
            return peor, ('a',) + arista
    # Fuera (o sobre un segmento): lo mas cercano del borde
    mejor, rasgo = _INF, None
    for k in range(n if n > 2 else 1):
        i, j = casco[k], casco[(k + 1) % n]
        d, t = _a_segmento(g, pts[i], pts[j])
        if d < mejor - 1e-15:
            mejor = d
            rasgo = (('v', i) if t <= 1e-9 else ('v', j) if t >= 1.0 - 1e-9
                     else ('a', i, j))
    return -mejor, rasgo


def _hay_mas_alla(g, pts, casco, rasgo, otros) -> bool:
    """Algun candidato fuera del poligono por el lado del CdG."""
    if not otros or rasgo is None:
        return False
    if rasgo[0] == 'a':
        a, b = pts[rasgo[1]], pts[rasgo[2]]
        es, el = b[0] - a[0], b[1] - a[1]
        n = math.hypot(es, el)
        lado = (es * (g[1] - a[1]) - el * (g[0] - a[0])) / n
        for s, lat in otros:
            d = (es * (lat - a[1]) - el * (s - a[0])) / n
            if len(casco) >= 3:
                if d < -1e-9:
                    return True
            elif abs(d) > 1e-9 and (abs(lado) < 1e-12 or d * lado > 0.0):
                return True
        return False
    p = pts[rasgo[1]]
    es, el = g[0] - p[0], g[1] - p[1]
    n = math.hypot(es, el)
    for s, lat in otros:
        if n < 1e-12:
            if math.hypot(s - p[0], lat - p[1]) > 1e-9:
                return True
        elif ((s - p[0]) * es + (lat - p[1]) * el) / n > 1e-9:
            return True
    return False


def _terna(cand, g, i, j, k):
    """(valor en g, b_s, b_l, baricentricas) si la terna contiene g."""
    si, li, hi_ = cand[i][0], cand[i][1], cand[i][2]
    a1, b1 = cand[j][0] - si, cand[j][1] - li
    a2, b2 = cand[k][0] - si, cand[k][1] - li
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-12:
        return None
    u, w = g[0] - si, g[1] - li
    lj = (u * b2 - a2 * w) / det
    lk = (a1 * w - u * b1) / det
    l0 = 1.0 - lj - lk
    if l0 < -1e-12 or lj < -1e-12 or lk < -1e-12:
        return None
    dj, dk = cand[j][2] - hi_, cand[k][2] - hi_
    return (l0 * hi_ + lj * cand[j][2] + lk * cand[k][2],
            (dj * b2 - dk * b1) / det, (a1 * dk - a2 * dj) / det, (l0, lj, lk))


def _plano(cand, g, previo, eps=1e-9):
    """(b_s, b_l, soporte) del plano de apoyo en g (§5.5), o None.

    cand: [(s, l, H, clave)]. El plano P(p) = a + b·(p - g) cumple P >= H en
    todos y deja P(g) minimo: es la terna que contiene g con mas valor en g.
    """
    n = len(cand)
    mejor = None
    if previo and len(previo) == 3:
        pos = {c[3]: i for i, c in enumerate(cand)}
        if all(k in pos for k in previo):
            ijk = tuple(pos[k] for k in previo)
            r = _terna(cand, g, *ijk)
            if r is not None and all(
                    r[0] + r[1] * (c[0] - g[0]) + r[2] * (c[1] - g[1]) >= c[2] - 1e-12
                    for c in cand):
                mejor = r + (ijk,)
    if mejor is None:
        for i in range(n - 2):
            for j in range(i + 1, n - 1):
                for k in range(j + 1, n):
                    r = _terna(cand, g, i, j, k)
                    if r is not None and (mejor is None or r[0] > mejor[0] + 1e-12):
                        mejor = r + ((i, j, k),)
    if mejor is None:
        # Todos alineados: una arista que pase por g
        par_ = None
        for i in range(n - 1):
            for j in range(i + 1, n):
                d, t = _a_segmento(g, cand[i], cand[j])
                if d < 1e-9:
                    val = cand[i][2] + t * (cand[j][2] - cand[i][2])
                    if par_ is None or val > par_[0]:
                        par_ = (val, i, j)
        if par_ is None:
            return None
        _, i, j = par_
        return _plano_arista(cand, g, i, j) + ((cand[i][3], cand[j][3]),)
    _, bs, bl, lam, (i, j, k) = mejor
    soporte = (cand[i][3], cand[j][3], cand[k][3])
    # g sobre una arista de la terna (a menos de eps): las dos caras empatan
    # y se promedian; si no, el plano salta de una a otra y el robot oscila
    cerca, arista = eps, None
    for u, w in ((j, k), (i, k), (i, j)):
        d, t = _a_segmento(g, cand[u], cand[w])
        if d < cerca and 0.0 < t < 1.0:
            cerca, arista = d, (u, w)
    if arista is not None:
        return _plano_arista(cand, g, *arista) + (soporte,)
    return bs, bl, soporte


def _plano_arista(cand, g, i, j):
    """Pendiente del plano por la arista i-j: la media de sus dos caras."""
    a, b = cand[i], cand[j]
    es, el = b[0] - a[0], b[1] - a[1]
    n = math.hypot(es, el)
    es, el = es / n, el / n
    ns, nl = -el, es
    m = (b[2] - a[2]) / n
    lo, hi = -_INF, _INF
    for c in cand:
        d = (c[0] - a[0]) * ns + (c[1] - a[1]) * nl
        if abs(d) < 1e-9:
            continue
        falta = c[2] - (a[2] + m * ((c[0] - a[0]) * es + (c[1] - a[1]) * el))
        if d > 0.0:
            lo = max(lo, falta / d)
        else:
            hi = min(hi, falta / d)
    if lo > -_INF and hi < _INF:
        t = 0.5 * (lo + hi)
    elif lo > -_INF:
        t = lo
    elif hi < _INF:
        t = hi
    else:
        t = 0.0
    return m * es + t * ns, m * el + t * nl


# ================================================================ giros

def _rot_zyx(yaw, th, ph):
    """R = Rz(yaw) Ry(cabeceo) Rx(balanceo)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(th), math.sin(th)
    cr, sr = math.cos(ph), math.sin(ph)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _rodrigues(a, ang):
    ax, ay, az = a
    c, s = math.cos(ang), math.sin(ang)
    t = 1.0 - c
    return ((c + ax * ax * t, ax * ay * t - az * s, ax * az * t + ay * s),
            (ay * ax * t + az * s, c + ay * ay * t, ay * az * t - ax * s),
            (az * ax * t - ay * s, az * ay * t + ax * s, c + az * az * t))


def _por(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _zyx(R):
    return (math.atan2(R[1][0], R[0][0]),
            math.asin(max(-1.0, min(1.0, -R[2][0]))),
            math.atan2(R[2][1], R[2][2]))


# ================================================================ reposo

def _apoyo(ctx, ev, g):
    """Candidatos, activos, su envolvente en planta y el margen del CdG."""
    par = ctx.par
    zr = ev.zreq
    ap = [c for c in ev.cons if c[_AP] and c[_H] >= zr - par.ventana_candidatos]
    # Una pared que toca con la normal hacia arriba tambien carga peso
    act = [c for c in ev.cons if c[_H] >= zr - par.tol_apoyo
           and (c[_AP] or c[_NZ] > 1e-6)]
    pts = [(c[_S], c[_L]) for c in act]
    casco = _envolvente(pts)
    margen, rasgo = _distancia(g, pts, casco)
    return ap, act, pts, casco, margen, rasgo


def _candidatos(ap, act, casco, par, zr):
    """Los vertices del poligono activo y los mas altos del resto (§5.4).
    Las paredes solo sostienen: su H no pasa de la de apoyo."""
    sel = [act[i] for i in casco]
    dentro = {id(c) for c in sel}
    resto = sorted((c for c in ap if id(c) not in dentro), key=lambda c: -c[_H])
    sel += resto[:max(0, par.max_candidatos - len(sel))]
    return [(c[_S], c[_L], min(c[_H], zr), c[_K]) for c in sel]


def _reposo(ctx, m, q, ev, presupuesto, iteraciones, z_min, soporte, colocando):
    """Cabeceo y balanceo por el plano de apoyo; vuelco si el CdG se sale.

    Devuelve (ev, soporte, angulo volcado). Al acabar m.z = max(H de apoyo).
    """
    par = ctx.par
    quieto = _QUIETO_COLOCAR if colocando else _QUIETO_PASO
    tope = _GIRO_COLOCAR if colocando else par.max_giro_iter
    girado = 0.0
    freno, antes = 1.0, (0.0, 0.0)
    m.z = max(ev.zreq, z_min)
    for _ in range(iteraciones):
        g = ctx.cdg(m, q)
        ap, act, pts, casco, margen, rasgo = _apoyo(ctx, ev, g)
        if not act:
            break
        plano = None
        eps = par.eps_equilibrio
        # colocar no tiene prisa: el plano primero, si alguna terna contiene g
        if colocando or margen > eps or (margen > -eps and _hay_mas_alla(
                g, pts, casco, rasgo,
                [(c[_S], c[_L]) for c in ap if c[_H] < ev.zreq - par.tol_apoyo])):
            plano = _plano(_candidatos(ap, act, casco, par, ev.zreq), g, soporte,
                           par.eps_equilibrio)
        if plano is not None:
            bs, bl, soporte = plano
            dth = max(-tope, min(tope, -par.relajacion * math.atan(bs)))
            dph = max(-tope, min(tope, par.relajacion * math.atan(bl)))
            # Si deshace la correccion anterior, oscila al cambiar los apoyos
            if dth * antes[0] + dph * antes[1] < 0.0:
                freno *= 0.5
            else:
                freno = min(1.0, 1.25 * freno)
            antes = (dth, dph)
            dth *= freno
            dph *= freno
            if abs(dth) + abs(dph) < quieto:
                break
            m.th += dth
            m.ph += dph
        else:
            if colocando:
                ang, ev = _vuelco_base(ctx, m, q, ev, act, pts, rasgo, g)
            else:
                resto = presupuesto - girado
                if resto <= 1e-12:
                    break
                ang, ev = _vuelco(ctx, m, q, ev, act, rasgo, g, resto,
                                  _BISECCIONES_VUELCO)
            girado += abs(ang)
            if abs(ang) < 1e-9:
                break
            m.z = max(ev.zreq, z_min)
            continue
        ev = ctx.evaluar(m, q)
        m.z = max(ev.zreq, z_min)
    return ev, soporte, girado


def _vuelco_base(ctx, m, q, ev, act, pts, rasgo, g):
    """El vuelco de colocar(): inclina sobre base_link hacia el CdG sin mover
    x, y ni yaw. La z la marcan las piezas que apoyan ahora (su contacto se
    desliza) y para en cuanto otra pieza toca o el plano de apoyo ve al CdG."""
    par = ctx.par
    grupo = {(c[_LI], c[_P]) for c in act}
    p = pts[rasgo[1]]
    if rasgo[0] == 'a':
        b = pts[rasgo[2]]
        es, el = b[1] - p[1], -(b[0] - p[0])            # normal de la arista
        if es * (g[0] - p[0]) + el * (g[1] - p[1]) < 0.0:
            es, el = -es, -el
    else:
        es, el = g[0] - p[0], g[1] - p[1]
    n = math.hypot(es, el)
    if n < 1e-12:
        return 0.0, ev
    es, el = es / n, el / n
    th0, ph0, z0 = m.th, m.ph, m.z

    def del_grupo(e):
        return max([-_INF] + [c[_H] for c in e.cons if (c[_LI], c[_P]) in grupo
                              and (c[_AP] or c[_NZ] > 1e-6)])

    def probar(d):
        m.th, m.ph = th0 + es * d, ph0 - el * d         # baja el lado del CdG
        m.z = z0
        e = ctx.evaluar(m, q)
        zg = del_grupo(e)
        if abs(zg - m.z) > 1e-4:                        # la z decide que es apoyo
            m.z = zg
            e = ctx.evaluar(m, q)
            zg = del_grupo(e)
        m.z = zg
        lim = zg + par.tol_penetra
        if any(c[_H] > lim and (c[_LI], c[_P]) not in grupo and (c[_AP] or c[_NZ] > 1e-6)
               for c in e.cons):
            return e, True
        gg = ctx.cdg(m, q)
        ap, ac, _, casco, _, _ = _apoyo(ctx, e, gg)
        return e, bool(ac) and _plano(_candidatos(ap, ac, casco, par, e.zreq), gg, ()) is not None

    giro = _VUELCO_COLOCAR
    e, para = probar(giro)
    if para:
        lo, hi, e_hi = 0.0, 1.0, e
        for _ in range(_BISECCIONES_COLOCAR):
            mid = 0.5 * (lo + hi)
            e, para = probar(giro * mid)
            if para:
                hi, e_hi = mid, e
            else:
                lo = mid
        giro *= hi                      # donde ya toca la pieza nueva: el plano la ve
        e = e_hi
    m.th, m.ph = th0 + es * giro, ph0 - el * giro
    m.z = max(e.zreq, del_grupo(e))
    return giro, e


def _vuelco(ctx, m, q, ev, act, rasgo, g, resto, mitades):
    """Gira sobre la arista (o el punto) de apoyo, hacia donde baja el CdG,
    hasta el primer contacto nuevo o hasta 'resto' rad. Devuelve (angulo, ev)."""
    par = ctx.par
    ca = act[rasgo[1]]
    xa, ya = ctx.mundo(m, ca[_S], ca[_L])
    P = (xa, ya, ca[_ZC])
    gx, gy = ctx.mundo(m, g[0], g[1])
    if rasgo[0] == 'a':
        cb = act[rasgo[2]]
        xb, yb = ctx.mundo(m, cb[_S], cb[_L])
        a = (xb - xa, yb - ya, cb[_ZC] - ca[_ZC])
    else:
        a = (ya - gy, gx - xa, 0.0)             # horizontal, perpendicular a G - P
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    if n < 1e-12:
        return 0.0, ev
    a = (a[0] / n, a[1] / n, a[2] / n)
    signo = -1.0 if a[0] * (gy - ya) - a[1] * (gx - xa) > 0.0 else 1.0
    # Los contactos sobre el eje ruedan con el giro: no son contactos nuevos
    ps, pl = ca[_S], ca[_L]
    if rasgo[0] == 'a':
        es, el = cb[_S] - ps, cb[_L] - pl
        ne = math.hypot(es, el)
        eje = [c[_K] for c in act if ne > 1e-12
               and abs(es * (c[_L] - pl) - el * (c[_S] - ps)) / ne < 1e-3]
    else:
        eje = [c[_K] for c in act if math.hypot(c[_S] - ps, c[_L] - pl) < 1e-3]
    eje = set(eje) | {ca[_K]}
    R0 = _rot_zyx(m.yaw, m.th, m.ph)
    d0 = (m.x - P[0], m.y - P[1], m.z - P[2])
    inicio = m.copia()
    pen0 = max([0.0] + [c[_H] - m.z for c in ev.cons if c[_K] not in eje])

    def probar(ang):
        Rr = _rodrigues(a, ang)
        m.yaw, m.th, m.ph = _zyx(_por(Rr, R0))
        m.x = P[0] + Rr[0][0] * d0[0] + Rr[0][1] * d0[1] + Rr[0][2] * d0[2]
        m.y = P[1] + Rr[1][0] * d0[0] + Rr[1][1] * d0[1] + Rr[1][2] * d0[2]
        m.z = P[2] + Rr[2][0] * d0[0] + Rr[2][1] * d0[1] + Rr[2][2] * d0[2]
        e = ctx.evaluar(m, q)
        pen = max([-_INF] + [c[_H] - m.z for c in e.cons if c[_K] not in eje])
        return e, pen > pen0 + par.tol_penetra

    e, choca = probar(signo * resto)
    if not choca:
        return signo * resto, e
    lo, hi = 0.0, 1.0
    bueno = None
    for _ in range(mitades):
        mid = 0.5 * (lo + hi)
        e, choca = probar(signo * resto * mid)
        if choca:
            hi = mid
        else:
            lo = mid
            bueno = (m.copia(), e)
    if bueno is None:
        m.poner(inicio)
        return 0.0, ev
    m.poner(bueno[0])
    return signo * resto * lo, bueno[1]


# ================================================================ paredes

def _empujar(ctx, m, q, ev, tope):
    """Saca las restricciones de PARED que penetran moviendo la base a lo
    largo del rumbo. Devuelve (ev, desplazamiento, restriccion, opuestos)."""
    lim = m.z + ctx.par.tol_penetra
    alante = atras = None
    for c in ev.cons:
        if not c[_AP] and c[_H] > lim:
            if c[_EM] > 0.0 and (alante is None or c[_EM] > alante[_EM]):
                alante = c
            elif c[_EM] < 0.0 and (atras is None or c[_EM] < atras[_EM]):
                atras = c
    if alante is None and atras is None:
        return ev, 0.0, None, False
    if alante is not None and atras is not None:
        c = alante if alante[_EM] >= -atras[_EM] else atras
        return ev, 0.0, c, True
    c = alante or atras
    if tope <= 0.0:
        return ev, 0.0, c, False
    d = math.copysign(min(abs(c[_EM]) + _EXTRA_EMPUJE, tope), c[_EM])
    m.x += d * math.cos(m.yaw)
    m.y += d * math.sin(m.yaw)
    return ctx.evaluar(m, q), d, c, False


def _penetra_pared(ctx, m, ev) -> bool:
    lim = m.z + ctx.par.tol_penetra
    return any(not c[_AP] and c[_H] > lim for c in ev.cons)


def _guardia(ctx, antes, despues, mov, dq, z):
    """(linea, pieza) cuya z requerida sube por encima de z mas de
    d·tan(ataque) + 2 mm, o None. mov = (dx, dy, dyaw) de la base; dq, lo que
    ha girado cada brazo. Lo que sube sin llegar a z no toca nada."""
    t = math.tan(ctx.par.ataque_banda)
    L = ctx.geo.orugas.largo
    plano = math.hypot(mov[0], mov[1])
    peor, exceso = None, 0.0
    for clave, h1 in despues.hp.items():
        h0 = antes.hp.get(clave)
        if h0 is None or h0 == -_INF or h1 == -_INF:
            continue
        p = clave[1]
        d = plano + abs(mov[2]) * ctx.radio[p] + (L * dq[p] if p != CHASIS else 0.0)
        e = h1 - max(h0, z) - (d * t + _HOLGURA_GUARDIA)
        if e > exceso:
            peor, exceso = clave, e
    return peor


# ================================================================ API

def colocar(x: float, y: float, yaw: float, elevaciones: Sequence[float], terreno,
            geo: GeometriaRobot, par: Parametros = Parametros()) -> Estado:
    """Posa el robot en (x, y, yaw) sin moverlo en planta (salvo 5 mm de empuje)."""
    q = tuple(float(e) for e in elevaciones)
    ctx = _Ctx(terreno, geo, par, {})
    m = _Mov(float(x), float(y), float(yaw), 0.0, 0.0, 0.0)
    ev = ctx.evaluar(m, q)
    for _ in range(3):                          # la z decide que es apoyo
        if ev.zreq == -_INF or abs(ev.zreq - m.z) < 1e-12:
            break
        m.z = ev.zreq
        ev = ctx.evaluar(m, q)
    ev, d, _, _ = _empujar(ctx, m, q, ev, par.empuje_max)
    if _penetra_pared(ctx, m, ev):
        # Puesto dentro de un obstaculo: lo que no saca el empuje lo sube encima
        encima = _Ctx(terreno, geo, replace(par, ataque_banda=math.pi,
                                            ataque_casco=math.pi), ctx.perfiles)
        ev = encima.evaluar(m, q)
        ev, soporte, _ = _reposo(encima, m, q, ev, 0.0, _ITER_COLOCAR, -_INF, (), True)
        ev = ctx.evaluar(m, q)
    else:
        ev, soporte, _ = _reposo(ctx, m, q, ev, 0.0, _ITER_COLOCAR, -_INF, (), True)
        ev, d2, _, _ = _empujar(ctx, m, q, ev, par.empuje_max - abs(d))
        if d2:
            ev, soporte, _ = _reposo(ctx, m, q, ev, 0.0, _ITER_COLOCAR, -_INF, soporte,
                                     True)
    volcado = abs(m.th) > par.vuelco or abs(m.ph) > par.vuelco
    return _hacer_estado(ctx, m, q, ev, soporte, None, False, volcado, False,
                         0.0, 0.0, None)


def paso(estado: Estado, v: float, w: float, elevaciones: Sequence[float], dt: float,
         terreno, geo: GeometriaRobot, par: Parametros = Parametros()) -> Estado:
    """Un tick: avance plano de las orugas (v m/s, w rad/s), bloqueo y reposo."""
    q1 = tuple(float(e) for e in elevaciones)
    dt = max(0.0, float(dt))
    if estado.volcado:
        return _congelado(estado, q1, v * dt)
    cache = estado.cache or {}
    mismo = cache.get('terreno') is terreno
    ctx = _Ctx(terreno, geo, par, dict(cache.get('perfiles', {})) if mismo else {})
    p = estado.pose
    m = _Mov(p.x, p.y, p.yaw, p.z, p.cabeceo, p.balanceo)
    q0 = tuple(float(e) for e in estado.elevaciones)
    ult = cache.get('ultima')
    if (mismo and ult is not None and ult[0] == p and ult[1] == q0
            and ult[2] is geo and ult[3] == par):
        ev = ult[4]
    else:
        ev = ctx.evaluar(m, q0)
    dqmax = max([0.0] + [abs(b - a) for a, b in zip(q0, q1)])
    n = max(1, math.ceil(max(abs(v) * dt, 0.75 * abs(w) * dt, 0.40 * dqmax)
                         / par.subpaso - 1e-9))
    dts = dt / n
    soporte = estado.soporte
    total = propuesto = avance = 0.0
    comido = [0.0] * len(PIEZAS)
    cayendo = atrapado = False
    choque = None
    for k in range(n):
        qa = tuple(a + (b - a) * k / n for a, b in zip(q0, q1))
        qb = tuple(a + (b - a) * (k + 1) / n for a, b in zip(q0, q1))
        x0, y0, yaw0, z0 = m.x, m.y, m.yaw, m.z
        # (b) traccion
        tr = [c[_NZ] for c in ev.cons if c[_AP] and c[_TR] and c[_P] != CHASIS
              and c[_H] >= m.z - par.tol_traccion]
        f = sum(tr) / len(tr) if tr else 1.0
        propuesto += v * f * dts
        mover = bool(tr) and (v != 0.0 or w != 0.0) and dts > 0.0
        tam = 0.0
        if mover:
            # (c) propuesta plana
            tam = (abs(v * f) + 0.75 * abs(w)) * dts
            total += tam
            m.x, m.y, m.yaw = integrar(x0, y0, yaw0, v * f, w, dts)
        ev1 = ctx.evaluar(m, qb)
        if mover:
            # (d) guardia contra el teletransporte
            dq = [abs(b - a) for a, b in zip(qa, qb)]

            def mov():
                return m.x - x0, m.y - y0, (m.yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi

            culpa = _guardia(ctx, ev, ev1, mov(), dq, z0)
            if culpa is not None:
                lo, hi = 0.0, 1.0
                ev_lo, ev_hi = None, ev1
                for _ in range(_BISECCIONES_GUARDIA):
                    mid = 0.5 * (lo + hi)
                    m.x, m.y, m.yaw = integrar(x0, y0, yaw0, v * f, w, dts * mid)
                    e = ctx.evaluar(m, qb)
                    c2 = _guardia(ctx, ev, e, mov(), dq, z0)
                    if c2 is not None:
                        hi, ev_hi, culpa = mid, e, c2
                    else:
                        lo, ev_lo = mid, e
                m.x, m.y, m.yaw = integrar(x0, y0, yaw0, v * f, w, dts * lo)
                ev1 = ev_lo if ev_lo is not None else ctx.evaluar(m, qb)
                comido[culpa[1]] += (1.0 - lo) * tam
                suyas = [c for c in ev_hi.cons if (c[_LI], c[_P]) == culpa]
                if suyas:
                    choque = max(suyas, key=lambda c: c[_H])
        # (e) empuje, antes del reposo
        ev1, d, c, opuestos = _empujar(ctx, m, qb, ev1, par.empuje_max)
        if opuestos:
            m.x, m.y, m.yaw = x0, y0, yaw0
            ev1 = ctx.evaluar(m, qb)
            comido[c[_P]] += tam
            atrapado = atrapado or _penetra_pared(ctx, m, ev1)
        elif c is not None and d and (v == 0.0 or d * v < 0.0):
            comido[c[_P]] += abs(d)
        # reposo
        z_min = z0 - par.caida_lin * dts
        ev1, soporte, girado = _reposo(ctx, m, qb, ev1, par.caida_ang * dts,
                                       par.iteraciones, z_min, soporte, False)
        cayendo = cayendo or girado > 1e-9
        # (e) empuje, despues del reposo
        ev1, d, c, opuestos = _empujar(ctx, m, qb, ev1, par.empuje_max)
        if opuestos:
            atrapado = atrapado or _penetra_pared(ctx, m, ev1)
        elif c is not None and d and (v == 0.0 or d * v < 0.0):
            comido[c[_P]] += abs(d)
        m.z = max(ev1.zreq, z_min)
        cayendo = cayendo or m.z > ev1.zreq + par.tol_apoyo
        avance += (m.x - x0) * math.cos(yaw0) + (m.y - y0) * math.sin(yaw0)
        ev = ev1
    volcado = abs(m.th) > par.vuelco or abs(m.ph) > par.vuelco
    bloqueado = None
    if total > 1e-12 and sum(comido) > 0.5 * total:
        bloqueado = PIEZAS[max(range(len(PIEZAS)), key=lambda i: comido[i])]
    return _hacer_estado(ctx, m, q1, ev, soporte, bloqueado, cayendo, volcado,
                         atrapado, avance, propuesto, choque if bloqueado else None)


def _congelado(estado: Estado, q, propuesto: float) -> Estado:
    return replace(estado, elevaciones=q, avance=0.0, propuesto=propuesto,
                   cayendo=False, bloqueado=None)


def _hacer_estado(ctx, m, q, ev, soporte, bloqueado, cayendo, volcado, atrapado,
                  avance, propuesto, choque) -> Estado:
    par = ctx.par
    z = m.z
    cy, sy = math.cos(m.yaw), math.sin(m.yaw)

    def contacto(c, pared=False):
        x, y = ctx.mundo(m, c[_S], c[_L])
        apoyo = c[_AP] and not pared
        return Contacto(PIEZAS[c[_P]], x, y, c[_ZC], (c[_NS] * cy, c[_NS] * sy, c[_NZ]),
                        math.acos(max(-1.0, min(1.0, c[_NZ]))),
                        'apoyo' if apoyo else 'pared',
                        bool(apoyo and c[_TR] and c[_P] != CHASIS), z - c[_H])

    g = ctx.cdg(m, q)
    ap, act, pts, casco, margen, _ = _apoyo(ctx, ev, g)
    n = len(ctx.pivotes)
    h_ap, h_todas = [-_INF] * n, [-_INF] * n
    for c in ev.cons:
        p = c[_P]
        if p != CHASIS:
            if c[_H] > h_todas[p]:
                h_todas[p] = c[_H]
            if c[_AP] and c[_H] > h_ap[p]:
                h_ap[p] = c[_H]
    holguras = tuple(max(0.0, z - (a if a > -_INF else t)) if max(a, t) > -_INF else _INF
                     for a, t in zip(h_ap, h_todas))
    lista = sorted(ap, key=lambda c: -c[_H])[:_MAX_CONTACTOS]
    lista += [c for c in ev.cons if not c[_AP] and c[_H] >= z - par.tol_traccion]
    contactos = tuple(contacto(c) for c in lista)
    if choque is not None:                      # contra lo que choca: tocando
        contactos += (replace(contacto(choque, pared=True), holgura=0.0),)
    poligono = tuple(ctx.mundo(m, act[i][_S], act[i][_L]) + (act[i][_ZC],) for i in casco)
    gx, gy = ctx.mundo(m, g[0], g[1])
    pose = Pose3D(m.x, m.y, m.yaw, m.z, m.th, m.ph)
    return Estado(
        pose=pose, elevaciones=q, contactos=contactos,
        apoya=tuple(h <= par.tol_apoyo for h in holguras), holguras=holguras,
        panza=any(c[_P] == CHASIS and c[_AP] and c[_H] >= z - par.tol_apoyo
                  for c in ev.cons),
        bloqueado=bloqueado,
        sin_traccion=not any(c[_AP] and c[_TR] and c[_P] != CHASIS
                             and c[_H] >= z - par.tol_traccion for c in ev.cons),
        cayendo=bool(cayendo), volcado=bool(volcado), atrapado=bool(atrapado),
        cdg=(gx, gy, g[2]), poligono=poligono,
        margen=margen if margen > -_INF else -_INF,
        avance=avance, propuesto=propuesto, soporte=tuple(soporte or ()),
        cache={'terreno': ctx.ter, 'perfiles': ctx.perfiles,
               'ultima': (pose, q, ctx.geo, par, ev)})
