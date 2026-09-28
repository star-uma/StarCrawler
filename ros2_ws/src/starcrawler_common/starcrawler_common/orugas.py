"""
orugas.py — el esquema del mando, compartido
============================================
Lo usan el teleop del DS4 (joy_logic) y el mando web (mando_web): los dos
traducen "par delantero sube", "inclinar adelante", un preset o el stick a
las mismas consignas. Si cambia aqui, cambia en los dos.

Convenio de ELEVACION: +1 = ese brazo sube. Orden {FR, FL, RR, RL}.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

N_ORUGAS = 4

# Incremento para SUBIR cada par
PAR_DELANTERO = (1, 1, 0, 0)   # FR, FL
PAR_TRASERO = (0, 0, 1, 1)     # RR, RL

# Presets de pose en grados de elevacion (225/180/135/90 de encoder en la
# referencia F.R. del TFG): X, O, cuadrado, triangulo del DS4
PRESETS_DEG = (-45.0, 0.0, 45.0, 90.0)

# Topes de /cmd_vel: los 40 dps del firmware con r=0.0764 m y L=0.524 m
MAX_LINEAL = 0.053          # m/s
MAX_ANGULAR = 0.20          # rad/s
FACTOR_LENTO = 0.5
S_PRESET = 0.5              # s manteniendo el preset para aplicarlo

INCLINACIONES = ('arriba', 'abajo', 'izq', 'der')


def signo(v: float) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)


def inclinacion(sentido: Optional[str]) -> List[int]:
    """Cruceta -> inclinar el conjunto. Es el modo 3 del TFG en ELEVACION.

    Al subir los brazos de un extremo, ese extremo del chasis pierde apoyo y
    baja. Equivalencias con los vectores en grados de encoder del TFG:
        arriba {-1,1,-1,1} -> {+1,+1,-1,-1}   abajo  { 1,-1, 1,-1} -> {-1,-1,+1,+1}
        izq    { 1,1,-1,-1} -> {-1,+1,-1,+1}   der    {-1,-1, 1, 1} -> {+1,-1,+1,-1}
    """
    if sentido == 'arriba':
        return [1, 1, -1, -1]     # suben delanteras -> inclinar adelante
    if sentido == 'abajo':
        return [-1, -1, 1, 1]     # suben traseras   -> inclinar atras
    if sentido == 'izq':
        return [-1, 1, -1, 1]     # suben las del lado izquierdo
    if sentido == 'der':
        return [1, -1, 1, -1]     # suben las del lado derecho
    return [0, 0, 0, 0]


def componer_incremento(delanteras: int, traseras: int,
                        inclinar: Sequence[int] = (0, 0, 0, 0)) -> List[int]:
    """Pares (+1 sube, -1 baja) mas inclinacion, sumados y saturados por oruga."""
    return [signo(delanteras * PAR_DELANTERO[i] + traseras * PAR_TRASERO[i]
                  + inclinar[i])
            for i in range(N_ORUGAS)]


def traccion(avance: float, giro: float, factor: float = 1.0,
             max_lineal: float = MAX_LINEAL,
             max_angular: float = MAX_ANGULAR) -> Tuple[float, float]:
    """Stick normalizado -> (lineal m/s, angular rad/s).

    avance + = adelante; giro + = a la DERECHA, que en ROS es angular.z
    negativo.
    """
    return (avance * max_lineal * factor, -giro * max_angular * factor)
