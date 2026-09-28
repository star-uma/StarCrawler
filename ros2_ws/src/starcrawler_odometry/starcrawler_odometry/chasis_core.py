"""
chasis_core.py — el chasis apoyado en sus cuatro orugas, sin ROS
================================================================
Modulo PURO, como odometry_core.py. Dada la elevacion de las orugas,
calcula como queda el chasis sobre suelo llano: altura, cabeceo y balanceo.
Es lo que publica chasis_node en las juntas virtuales del URDF.

Cada oruga es la banda alrededor de dos poleas: la activa (radio R1) en el
pivote y la pasiva (R2) en la punta, a L del pivote. Su punto mas bajo es
el de una de las dos, asi que el pivote queda a
    d = max(R1, R2 - L sin(elevacion vista desde el suelo))
El chasis baja hasta que su centro no puede bajar mas sin que una oruga
atraviese el suelo. Con los pivotes en rectangulo eso es la mayor de las
medias de las dos diagonales; si una diagonal queda por encima, las otras
dos orugas se levantan lo mismo (equilibrio sobre la diagonal).

La elevacion vista desde el suelo depende del cabeceo y el cabeceo de la
elevacion: se itera. Convenio REP-103: cabeceo + = morro abajo, balanceo + =
lado izquierdo arriba.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Sequence, Tuple

# Orden de los vectores en todo el proyecto
JUNTAS_ORUGA = ('crawler_fr_joint', 'crawler_fl_joint',
                'crawler_rr_joint', 'crawler_rl_joint')
JUNTAS_CHASIS = ('chassis_lift_joint', 'chassis_pitch_joint',
                 'chassis_roll_joint')

_ITERACIONES = 200
_RELAJACION = 0.5
_TOLERANCIA = 1e-10


@dataclass
class Geometria:
    pivotes: Sequence[Tuple[float, float]]  # (x, y) en base_link, {FR, FL, RR, RL}
    radio_polea: float                      # R1, polea activa en el pivote
    radio_punta: float                      # R2, polea pasiva
    largo: float                            # L, entre ejes de las poleas
    altura_reposo: float                    # pivote sobre el suelo, orugas en llano


@dataclass
class Pose:
    altura: float                           # pivotes (base_link) sobre el suelo
    cabeceo: float
    balanceo: float
    holguras: List[float]                   # de cada oruga al suelo; 0 = apoya


def geometria_desde_urdf(xml: str) -> Geometria:
    """Lee pivotes, poleas y altura de reposo del URDF ya expandido."""
    robot = ET.fromstring(xml)
    juntas = {j.get('name'): j for j in robot.findall('joint')}
    links = {l.get('name'): l for l in robot.findall('link')}

    def xyz(elem):
        origen = elem.find('origin') if elem is not None else None
        texto = origen.get('xyz') if origen is not None else None
        return [float(v) for v in texto.split()] if texto else [0.0, 0.0, 0.0]

    pivotes = []
    poleas = []
    for nombre in JUNTAS_ORUGA:
        if nombre not in juntas:
            raise ValueError('Falta %s en el URDF' % nombre)
        x, y, _ = xyz(juntas[nombre])
        pivotes.append((x, y))
        hijo = juntas[nombre].find('child').get('link')
        cilindros = []
        for vis in links[hijo].findall('visual'):
            cil = vis.find('geometry/cylinder')
            if cil is not None:
                cilindros.append((abs(xyz(vis)[0]), float(cil.get('radius'))))
        if len(cilindros) != 2:
            raise ValueError('%s necesita dos poleas (cilindros), tiene %d'
                             % (hijo, len(cilindros)))
        cilindros.sort()
        poleas.append((cilindros[0][1], cilindros[1][1], cilindros[1][0]))

    if any(abs(p - q) > 1e-9 for par in poleas for p, q in zip(par, poleas[0])):
        raise ValueError('Las cuatro orugas deben tener las mismas poleas')
    xs = {round(abs(x), 9) for x, _ in pivotes}
    ys = {round(abs(y), 9) for _, y in pivotes}
    if len(xs) != 1 or len(ys) != 1:
        raise ValueError('Los pivotes deben formar un rectangulo centrado')
    if 'chassis_lift_joint' not in juntas:
        raise ValueError('Falta chassis_lift_joint en el URDF')

    r1, r2, largo = poleas[0]
    return Geometria(pivotes, r1, r2, largo, xyz(juntas['chassis_lift_joint'])[2])


def altura_pivote(elevacion_suelo: float, geo: Geometria) -> float:
    """Altura del pivote de una oruga que apoya, con su elevacion vista desde el suelo."""
    return max(geo.radio_polea,
               geo.radio_punta - geo.largo * math.sin(elevacion_suelo))


def _limitar(v: float) -> float:
    return max(-1.0, min(1.0, v))


def pose_chasis(elevaciones: Sequence[float], geo: Geometria) -> Pose:
    """Elevaciones {FR, FL, RR, RL} en rad (+ = brazo levantado) -> Pose."""
    cabeceo = balanceo = 0.0
    x2 = sum(x * x for x, _ in geo.pivotes)
    y2 = sum(y * y for _, y in geo.pivotes)
    for _ in range(_ITERACIONES):
        requerida = _alturas_requeridas(elevaciones, geo, cabeceo, balanceo)
        # Media de cada diagonal: FR-RL y FL-RR
        a = (requerida[0] + requerida[3]) / 2.0
        b = (requerida[1] + requerida[2]) / 2.0
        if a >= b:
            z = [requerida[0], requerida[1] + a - b, requerida[2] + a - b, requerida[3]]
        else:
            z = [requerida[0] + b - a, requerida[1], requerida[2], requerida[3] + b - a]

        gx = sum(x * zi for (x, _), zi in zip(geo.pivotes, z)) / x2
        gy = sum(y * zi for (_, y), zi in zip(geo.pivotes, z)) / y2
        nuevo_cabeceo = -math.asin(_limitar(gx))
        nuevo_balanceo = math.asin(_limitar(gy / math.cos(nuevo_cabeceo)))
        cambio = abs(nuevo_cabeceo - cabeceo) + abs(nuevo_balanceo - balanceo)
        cabeceo += _RELAJACION * (nuevo_cabeceo - cabeceo)
        balanceo += _RELAJACION * (nuevo_balanceo - balanceo)
        if cambio < _TOLERANCIA:
            break

    # Con el cabeceo y el balanceo ya quietos, la altura de la que apoya
    requerida = _alturas_requeridas(elevaciones, geo, cabeceo, balanceo)
    relativa = [_altura_relativa(x, y, cabeceo, balanceo) for x, y in geo.pivotes]
    altura = max(req - rel for req, rel in zip(requerida, relativa))
    holguras = [altura + rel - req for req, rel in zip(requerida, relativa)]
    return Pose(altura, cabeceo, balanceo, holguras)


def _altura_relativa(x: float, y: float, cabeceo: float, balanceo: float) -> float:
    """Altura de un pivote sobre el centro del chasis: Ry(cabeceo) Rx(balanceo)."""
    return -x * math.sin(cabeceo) + y * math.sin(balanceo) * math.cos(cabeceo)


def _alturas_requeridas(elevaciones: Sequence[float], geo: Geometria,
                        cabeceo: float, balanceo: float) -> List[float]:
    # El morro abajo baja la punta de las delanteras y sube la de las traseras
    return [altura_pivote(e - cabeceo if x > 0 else e + cabeceo, geo)
            * math.cos(balanceo)
            for e, (x, _) in zip(elevaciones, geo.pivotes)]
