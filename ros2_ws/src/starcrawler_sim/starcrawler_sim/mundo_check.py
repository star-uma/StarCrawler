"""
mundo_check.py — valida un mundo y resume lo que tiene
======================================================
Modulo PURO, sin ROS:

    ros2 run starcrawler_sim mundo_check practica_rrl [otro.yaml ...]

Imprime el resumen de cada mundo y los avisos: saltos verticales que el
robot no puede trepar, campos de escalones que rompen la regla 1 de Jacoff
et al., escaleras que acaban en un cortado, elementos fuera de 'limites',
solapes dentro de un carril y rotulos que RViz no sabe dibujar.
Sale con 1 si algun mundo no carga.
"""
from __future__ import annotations

import math
import sys
from typing import List, Optional, Sequence, Tuple

from starcrawler_sim.mundo_core import (
    ErrorMundo,
    Mundo,
    Solido,
    cargar_fichero,
    resolver_ruta,
)

SALTO_MAX = 0.40            # por encima del primer contacto maximo (0,4017)
UNIDADES_MAX = 2            # regla 1 de Jacoff et al.
PASO_MUESTREO = 0.05
DESPEGUE = 0.002            # a cada lado de una cara, para medir el salto
TOL_LIMITES = 0.001
TOL_SOLAPE = 0.001

USO = 'mundo_check <nombre en mundos/ o fichero.yaml> [...]'


def _saltos(mundo: Mundo) -> List[str]:
    """Caras de mas de SALTO_MAX que se encuentra el robot avanzando por su carril."""
    rumbos = {c.nombre: c.rumbo for c in mundo.carriles}
    out = []
    for el in mundo.elementos:
        rumbo = rumbos.get(el.grupo, -mundo.inicio[2])
        ux, uy = math.cos(rumbo), math.sin(rumbo)
        peor = 0.0
        for k in el.solidos:
            so = mundo.solidos[k]
            e = so.esquinas()
            if max(so.techo(x, y) for x, y in e) <= SALTO_MAX:
                continue
            for i in range(4):
                (xa, ya), (xb, yb) = e[i], e[(i + 1) % 4]
                lado = math.hypot(xb - xa, yb - ya)
                nx, ny = (yb - ya) / lado, (xa - xb) / lado     # hacia fuera
                if abs(nx * ux + ny * uy) < 0.5:
                    continue                                    # cara lateral
                n = max(2, math.ceil(lado / PASO_MUESTREO))
                for q in range(n):
                    f = (q + 0.5) / n
                    x, y = xa + (xb - xa) * f, ya + (yb - ya) * f
                    # Una cara contra una pared no se alcanza
                    salto = (mundo.altura(x - nx * DESPEGUE, y - ny * DESPEGUE, False)
                             - mundo.altura(x + nx * DESPEGUE, y + ny * DESPEGUE))
                    peor = max(peor, salto)
        if peor > SALTO_MAX + 1e-9:
            out.append('%s: salto vertical de %.3f m (mas de %.2f: el primer contacto '
                       'llega como mucho a 0.4017)' % (el.donde, peor, SALTO_MAX))
    return out


def _regla_campos(mundo: Mundo) -> List[str]:
    out = []
    for el in mundo.elementos:
        m = el.matriz
        if not m:
            continue
        malos = []
        for i, fila in enumerate(m):
            for j, a in enumerate(fila):
                for di, dj in ((1, 0), (0, 1)):
                    if i + di < len(m) and j + dj < len(fila):
                        if abs(m[i + di][j + dj] - a) > UNIDADES_MAX:
                            malos.append((i, j))
        if malos:
            i, j = malos[0]
            out.append('%s: %d saltos de mas de %d unidades entre celdas vecinas, el '
                       'primero en la fila %d, columna %d (regla 1 de Jacoff et al.)'
                       % (el.donde, len(malos), UNIDADES_MAX, i + 1, j + 1))
    return out


def _cortados(mundo: Mundo) -> List[str]:
    return ['%s: sin bajada, tras el rellano queda un cortado de %.2f m'
            % (el.donde, el.cortado) for el in mundo.elementos if el.cortado > 0]


def _fuera_de_limites(mundo: Mundo) -> List[str]:
    if not mundo.limites:
        return []
    (x0, y0), (x1, y1) = mundo.limites
    out = []
    for el in mundo.elementos:
        for k in el.solidos:
            if any(x < x0 - TOL_LIMITES or x > x1 + TOL_LIMITES
                   or y < y0 - TOL_LIMITES or y > y1 + TOL_LIMITES
                   for x, y in (mundo.a_fichero(*p) for p in mundo.solidos[k].esquinas())):
                out.append("%s: fuera de 'limites'" % el.donde)
                break
    return out


def _proyeccion(e: Sequence[Tuple[float, float]], ax: float, ay: float):
    v = [x * ax + y * ay for x, y in e]
    return min(v), max(v)


def _penetracion(a: Solido, b: Solido) -> float:
    """Solape de dos rectangulos en planta por ejes separadores (<= 0: no solapan)."""
    ea, eb = a.esquinas(), b.esquinas()
    peor = math.inf
    for e in (ea, eb):
        for i in (0, 1):
            dx, dy = e[i + 1][0] - e[i][0], e[i + 1][1] - e[i][1]
            n = math.hypot(dx, dy)
            ax, ay = -dy / n, dx / n
            amin, amax = _proyeccion(ea, ax, ay)
            bmin, bmax = _proyeccion(eb, ax, ay)
            peor = min(peor, min(amax, bmax) - max(amin, bmin))
    return peor


def _caja(mundo: Mundo, indices) -> Optional[Tuple[float, float, float, float]]:
    pts = [p for k in indices for p in mundo.solidos[k].esquinas()]
    if not pts:
        return None
    return (min(p[0] for p in pts), min(p[1] for p in pts),
            max(p[0] for p in pts), max(p[1] for p in pts))


def _solapes(mundo: Mundo) -> List[str]:
    out = []
    els = [(el, _caja(mundo, el.solidos)) for el in mundo.elementos]
    for n, (ea, ca) in enumerate(els):
        for eb, cb in els[n + 1:]:
            if ea.grupo != eb.grupo or ca is None or cb is None:
                continue
            if (ca[2] < cb[0] + TOL_SOLAPE or cb[2] < ca[0] + TOL_SOLAPE
                    or ca[3] < cb[1] + TOL_SOLAPE or cb[3] < ca[1] + TOL_SOLAPE):
                continue
            if any(_penetracion(mundo.solidos[i], mundo.solidos[j]) > TOL_SOLAPE
                   for i in ea.solidos for j in eb.solidos):
                out.append('%s: se solapa con el elemento %d (%s)'
                           % (ea.donde, eb.indice, eb.tipo))
    return out


def _rotulos(mundo: Mundo) -> List[str]:
    textos = [c.titulo for c in mundo.carriles] + [z.nombre for z in mundo.zonas]
    return ['rotulo %r con caracteres fuera de ASCII: RViz no los dibuja (la web si)' % t
            for t in textos if not t.isascii()]


def avisos(mundo: Mundo) -> List[str]:
    """Avisos de mundo_check (los de mundo_core ya van en mundo.resumen())."""
    return (_saltos(mundo) + _regla_campos(mundo) + _cortados(mundo)
            + _fuera_de_limites(mundo) + _solapes(mundo) + _rotulos(mundo))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in ('-h', '--help'):
        print('uso: ' + USO)
        return 0 if args else 2
    codigo = 0
    for nombre in args:
        try:
            ruta = resolver_ruta(nombre)
            mundo = cargar_fichero(ruta)
        except ErrorMundo as e:
            print('ERROR: %s' % e)
            codigo = 1
            continue
        print(ruta)
        print(mundo.resumen())
        for a in avisos(mundo):
            print('  aviso: ' + a)
    return codigo


if __name__ == '__main__':
    sys.exit(main())
