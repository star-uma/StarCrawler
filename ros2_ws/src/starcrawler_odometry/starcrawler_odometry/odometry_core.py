"""
odometry_core.py — odometria de orugas, sin ROS
===============================================
Modulo PURO: no importa rclpy ni nada de ROS, igual que protocol.py y
joy_logic.py. Asi se prueba entero en el PC (ver test/) y se puede llevar
a otro sitio sin arrastrar el grafo.

Convenio: velocidades de banda en rad/s en el eje de salida del RMD, tal
como las publica /starcrawler/state.
"""
from __future__ import annotations

import math
from typing import Tuple


def velocidades_del_robot(vel_izq_rad_s: float,
                          vel_der_rad_s: float,
                          radio_polea_m: float,
                          separacion_vias_m: float) -> Tuple[float, float]:
    """Cinematica directa de un diferencial.

    Devuelve (v, w): velocidad lineal en m/s y angular en rad/s.
    """
    v_izq = vel_izq_rad_s * radio_polea_m
    v_der = vel_der_rad_s * radio_polea_m

    v = (v_der + v_izq) / 2.0
    w = (v_der - v_izq) / separacion_vias_m
    return v, w


def integrar(x: float, y: float, th: float,
             v: float, w: float, dt: float) -> Tuple[float, float, float]:
    """Integra la pose un paso de tiempo.

    Para w pequeno se usa la aproximacion recta (Euler); por encima de ese
    umbral se integra el arco de circunferencia, que con giros cerrados y
    dt de 20 ms da bastante menos deriva.
    """
    if dt <= 0.0:
        return x, y, th

    th_nuevo = th + w * dt

    if abs(w) < 1e-6:
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
    else:
        radio = v / w
        x += radio * (math.sin(th_nuevo) - math.sin(th))
        y -= radio * (math.cos(th_nuevo) - math.cos(th))

    return x, y, normalizar_angulo(th_nuevo)


def normalizar_angulo(th: float) -> float:
    """Lleva el angulo al intervalo [-pi, pi], que es lo que da atan2.

    Ojo: -pi y +pi son el mismo angulo y atan2 puede devolver
    cualquiera de los dos segun el signo del cero. No compares el
    valor devuelto por igualdad contra uno de los dos.
    """
    return math.atan2(math.sin(th), math.cos(th))


def cuaternion_de_yaw(th: float) -> Tuple[float, float, float, float]:
    """Cuaternion (x, y, z, w) de una rotacion solo en yaw.

    Se calcula aqui para no depender de tf_transformations, que no viene
    en la instalacion base de ROS 2 y obliga a un apt install extra.
    """
    return (0.0, 0.0, math.sin(th / 2.0), math.cos(th / 2.0))
