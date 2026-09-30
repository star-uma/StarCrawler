"""medir_contacto.py — lo que tarda un tick de contacto_core (§5.10)

Escena de la escalera: mundos/escalera.yaml cargado con mundo_core (o la de
terreno_prueba con --prueba), 3000 ticks a 50 Hz con el guion de la escalera:
delanteras a 45 y, al pasar de 15 grados de morro arriba, los cuatro a -6.
Objetivo en WSL con Python 3.10: media < 5 ms y p99 < 15 ms; si no, bajar
rate_hz de mundo_node a 25. No es un test (no empieza por test_): el reloj
del WSL va a 0,92x y no conviene en colcon test.

    python3 test/medir_contacto.py [--ticks N] [--prueba]
"""
import argparse
import math
import os
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [AQUI, os.path.join(AQUI, '..')]

from starcrawler_sim import contacto_core as cc  # noqa: E402
from terreno_prueba import DT, GEO_PRUEBA, V, escalera, mover_brazos  # noqa: E402

MEDIA_MS, P99_MS = 5.0, 15.0


def terreno(prueba: bool):
    if not prueba:
        try:
            from starcrawler_sim import mundo_core
            ruta = os.path.join(AQUI, '..', 'mundos', 'escalera.yaml')
            return mundo_core.cargar_fichero(ruta), 'mundos/escalera.yaml (mundo_core)'
        except Exception as e:  # sin la parte A: el de prueba
            print('No cargo escalera.yaml (%s): uso el terreno de prueba' % e)
    return escalera(4, 0.20, 0.25, x=1.2), 'terreno_prueba.escalera(4, 0.20, 0.25)'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--ticks', type=int, default=3000)
    ap.add_argument('--prueba', action='store_true', help='terreno de prueba')
    args = ap.parse_args(argv)

    ter, nombre = terreno(args.prueba)
    q = tuple(math.radians(a) for a in (45, 45, 0, 0))
    objetivo = q
    e = cc.colocar(0.0, 0.0, 0.0, q, ter, GEO_PRUEBA)
    cambio = None
    tiempos = []
    z_max = 0.0
    for k in range(args.ticks):
        if cambio is None and e.pose.cabeceo < math.radians(-15):
            cambio = k * DT
            objetivo = tuple(math.radians(a) for a in (-6, -6, -6, -6))
        q = mover_brazos(q, objetivo)
        t0 = time.perf_counter()
        e = cc.paso(e, V, 0.0, q, DT, ter, GEO_PRUEBA)
        tiempos.append((time.perf_counter() - t0) * 1e3)
        z_max = max(z_max, e.pose.z)
    orden = sorted(tiempos)
    media = sum(tiempos) / len(tiempos)
    p99 = orden[min(len(orden) - 1, int(0.99 * len(orden)))]
    p = e.pose
    print('Escena: %s, %d ticks (%.0f s a 50 Hz), Python %s'
          % (nombre, args.ticks, args.ticks * DT, sys.version.split()[0]))
    print('Brazos a -6 en t = %s s; z maxima %.3f m; final x = %.3f z = %.3f cabeceo = %.1f'
          % ('%.1f' % cambio if cambio is not None else '-', z_max, p.x, p.z,
             math.degrees(p.cabeceo)))
    print('Tick: media %.2f ms, mediana %.2f, p99 %.2f, maximo %.2f (objetivo %.0f / %.0f)'
          % (media, orden[len(orden) // 2], p99, orden[-1], MEDIA_MS, P99_MS))
    bien = media < MEDIA_MS and p99 < P99_MS
    print('OK' if bien else 'NO LLEGA: bajar rate_hz de mundo_node a 25')
    return 0 if bien else 1


if __name__ == '__main__':
    sys.exit(main())
