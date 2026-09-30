"""Terreno de prueba para contacto_core: cajas y rampas alineadas con los ejes,
con perfil exacto, y la geometria fija GEO_PRUEBA (los numeros del URDF de
hoy), para que los tests no dependan de mundo_core ni del xacro."""
import math

from starcrawler_odometry.chasis_core import Geometria
from starcrawler_sim.contacto_core import GeometriaRobot

R1, R2, L = 0.0764, 0.0415, 0.36
PIVOTES = [(0.30, -0.24), (0.30, 0.24), (-0.30, -0.24), (-0.30, 0.24)]
GEO_PRUEBA = GeometriaRobot(Geometria(PIVOTES, R1, R2, L, R1),
                            (0.60, 0.40, 0.10), 0.08, 20.0, 3.0)
BETA = math.asin((R1 - R2) / L)
V = math.radians(40.0) * R1             # 40 dps en la polea activa: 0,0533 m/s
VEL_BRAZO = math.radians(4.69)          # rad/s, la del firmware
DT = 0.02
ANCHO = 5.0                             # medio ancho de lo que no dice otra cosa


class Terreno:
    """Maximo de z = 0 y de los techos de cajas y rampas (bordes cerrados)."""

    def __init__(self, piezas=()):
        # (x0, x1, y0, y1, a, bx, by): techo z = a + bx*x + by*y
        self.piezas = []
        for p in piezas:
            self.anadir(*p)

    def anadir(self, x0, x1, y0, y1, a, bx=0.0, by=0.0):
        self.piezas.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1),
                            a, bx, by))
        return self

    def caja(self, x0, x1, alto, y0=-ANCHO, y1=ANCHO):
        return self.anadir(x0, x1, y0, y1, alto)

    def rampa_x(self, x0, x1, z0, z1, y0=-ANCHO, y1=ANCHO):
        m = (z1 - z0) / (x1 - x0)
        return self.anadir(x0, x1, y0, y1, z0 - m * x0, m, 0.0)

    def altura(self, x, y):
        z = 0.0
        for x0, x1, y0, y1, a, bx, by in self.piezas:
            if x0 <= x <= x1 and y0 <= y <= y1:
                z = max(z, a + bx * x + by * y)
        return z

    def perfil(self, x0, y0, ux, uy, s0, s1):
        tramos = [(s0, s1, 0.0, 0.0)]
        for a0, a1, b0, b1, a, bx, by in self.piezas:
            lo, hi = s0, s1
            for p, u, c0, c1 in ((x0, ux, a0, a1), (y0, uy, b0, b1)):
                if abs(u) < 1e-12:
                    if p < c0 or p > c1:
                        lo, hi = 1.0, 0.0
                else:
                    t0, t1 = sorted(((c0 - p) / u, (c1 - p) / u))
                    lo, hi = max(lo, t0), min(hi, t1)
            if hi - lo > 1e-12:
                tramos.append((lo, hi, a + bx * x0 + by * y0, bx * ux + by * uy))
        cortes = {s0, s1}
        for lo, hi, _, _ in tramos:
            cortes.update((lo, hi))
        for i, (lo1, hi1, p1, m1) in enumerate(tramos):
            for lo2, hi2, p2, m2 in tramos[i + 1:]:
                if abs(m1 - m2) > 1e-12:
                    s = (p2 - p1) / (m1 - m2)
                    if max(lo1, lo2) < s < min(hi1, hi2):
                        cortes.add(s)
        cortes = sorted(cortes)
        pts = []
        for sa, sb in zip(cortes, cortes[1:]):
            if sb - sa < 1e-12:
                continue
            sm = 0.5 * (sa + sb)
            p, m = max(((p, m) for lo, hi, p, m in tramos if lo <= sm <= hi),
                       key=lambda t: t[0] + t[1] * sm)
            za, zb = p + m * sa, p + m * sb
            if not pts or abs(pts[-1][1] - za) > 1e-12:
                pts.append((sa, za))
            pts.append((sb, zb))
        out = [pts[0]]
        for k in range(1, len(pts) - 1):
            (sa, za), (sb, zb), (sc, zc) = out[-1], pts[k], pts[k + 1]
            if (sb - sa > 1e-12 and sc - sb > 1e-12
                    and abs(za + (zc - za) * (sb - sa) / (sc - sa) - zb) < 1e-12):
                continue
            out.append(pts[k])
        out.append(pts[-1])
        return out


# --- Escenas ---------------------------------------------------------------

def llano():
    return Terreno()


def escalon(h, x=1.0, largo=10.0):
    return Terreno().caja(x, x + largo, h)


def bajada(h, x=1.0):
    """Plataforma de alto h hasta x y suelo despues."""
    return Terreno().caja(-10.0, x, h)


def pared(x=1.0, alto=1.0, grosor=0.5):
    return Terreno().caja(x, x + grosor, alto)


def rampa(grados, largo=1.5, x=1.0):
    h = largo * math.tan(math.radians(grados))
    return Terreno().rampa_x(x, x + largo, 0.0, h).caja(x + largo, x + largo + 10.0, h)


def escalera(n, tabica, huella, x=1.0):
    """n tabicas desde x y rellano largo arriba, como escenas5.py."""
    t = Terreno()
    for k in range(n):
        fin = x + k * huella + (huella if k < n - 1 else 10.0)
        t.caja(x + k * huella, fin, (k + 1) * tabica)
    return t


def mover_brazos(q, objetivo, dt=DT):
    """Brazos hacia su objetivo a la velocidad del firmware."""
    paso = VEL_BRAZO * dt
    return tuple(a + max(-paso, min(paso, b - a)) for a, b in zip(q, objetivo))


def morro(estado, geo=GEO_PRUEBA):
    """Altura de los pivotes delanteros en el plano central."""
    p = estado.pose
    return p.z - geo.orugas.pivotes[0][0] * math.sin(p.cabeceo)
