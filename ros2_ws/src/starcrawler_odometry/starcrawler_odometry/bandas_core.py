"""Juntas de los tacos de las orugas (solo visual).

Los tacos van equiespaciados: basta desplazarlos lo que avanza la banda
modulo un paso para que parezca que la banda gira. Cada brazo tiene cuatro
juntas (tramo de arriba, tramo de abajo y los arcos de las dos poleas) y el
limite superior de cada una en el URDF es su periodo.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

BRAZOS = ('crawler_fr', 'crawler_fl', 'crawler_rr', 'crawler_rl')
IZQUIERDA = (False, True, False, True)
PATRON = re.compile(r'^(crawler_(?:fr|fl|rr|rl))_tacos_\w+_joint$')


def juntas_de_tacos(xml: str) -> Dict[str, List[Tuple[str, float]]]:
    """{brazo: [(junta, periodo), ...]} de las juntas de tacos del URDF."""
    salida: Dict[str, List[Tuple[str, float]]] = {b: [] for b in BRAZOS}
    for j in ET.fromstring(xml).findall('joint'):
        m = PATRON.match(j.get('name', ''))
        limite = j.find('limit')
        if m and limite is not None and float(limite.get('upper', 0)) > 0:
            salida[m.group(1)].append((j.get('name'), float(limite.get('upper'))))
    return salida


class Bandas:
    """Avance de cada banda y posicion de sus juntas de tacos."""

    def __init__(self, juntas: Dict[str, List[Tuple[str, float]]]):
        self.juntas = juntas
        # El paso lineal de cada brazo: el periodo de sus tramos (prismaticas)
        self.paso = {}
        for b, lista in juntas.items():
            tramos = [p for n, p in lista if n.endswith(('_arriba_joint', '_abajo_joint'))]
            self.paso[b] = tramos[0] if tramos else None
        self.avance = [0.0] * 4

    def avanzar(self, v_izq: float, v_der: float, dt: float) -> None:
        """v en m/s de la superficie de la banda de cada lado."""
        for i in range(4):
            v = v_izq if IZQUIERDA[i] else v_der
            if math.isfinite(v):
                self.avance[i] += v * dt

    def posiciones(self) -> Tuple[List[str], List[float]]:
        nombres, valores = [], []
        for i, b in enumerate(BRAZOS):
            paso = self.paso.get(b)
            if not paso:
                continue
            fase = math.fmod(self.avance[i], paso) / paso
            if fase < 0.0:
                fase += 1.0
            for nombre, periodo in self.juntas[b]:
                nombres.append(nombre)
                valores.append(fase * periodo)
        return nombres, valores

