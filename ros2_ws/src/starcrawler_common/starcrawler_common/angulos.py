"""
angulos.py — la traduccion entre los dos espacios de angulos
============================================================
Hay dos convenios en el robot y una unica traduccion entre ellos. Esta.

  Grados de encoder    lo que mide el AS5600 y usa el firmware.
                       180 = oruga horizontal. FL y RR van ESPEJADAS.

  Radianes de elevacion  lo que se publica en ROS. Positivo = brazo
                       LEVANTADO, 0 = horizontal, igual en las cuatro.

El espejado es la parte que se olvida: por eso vertical arriba es
{90, 270, 270, 90} en encoder para {FR, FL, RR, RL}, como documenta el
TFG, y en cambio es {+90, +90, +90, +90} en elevacion.

Vive en su propio paquete a proposito. Estaba copiada en el simulador y
en la GUI, y un error de signo aqui hace que el robot mueva los brazos al
reves: conviene que haya una sola copia y que este probada.

(El firmware de micro-ROS tiene la suya en C, en app.c. Esa no se puede
compartir, pero es la misma regla: si cambia el convenio, van las dos.)
"""
from __future__ import annotations

import math

N_ORUGAS = 4

# Orden en todos los vectores del proyecto: {FR, FL, RR, RL}
NOMBRES = ('FR', 'FL', 'RR', 'RL')


def es_espejada(i: int) -> bool:
    """FL y RR van montadas en espejo respecto a FR y RL."""
    return i == 1 or i == 2


def elevacion_a_encoder_deg(i: int, elevacion_rad: float) -> float:
    """Radianes de elevacion (+ = brazo levantado) -> grados de encoder."""
    d = math.degrees(elevacion_rad)
    return (180.0 + d) if es_espejada(i) else (180.0 - d)


def encoder_a_elevacion_rad(i: int, encoder_deg: float) -> float:
    """Grados de encoder -> radianes de elevacion."""
    d = (encoder_deg - 180.0) if es_espejada(i) else (180.0 - encoder_deg)
    return math.radians(d)
