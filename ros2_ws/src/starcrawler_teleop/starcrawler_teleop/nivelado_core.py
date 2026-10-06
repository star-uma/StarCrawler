"""
nivelado_core.py — nivelado automatico del chasis con los brazos, sin ROS
=========================================================================
Con el cabeceo y el balanceo del chasis (IMU) calcula a que altura queda cada
esquina y mueve los brazos: la esquina que queda baja baja su brazo (empuja el
chasis hacia arriba) y la que queda alta recoge su brazo si estaba empujando.
Un brazo nunca se levanta por encima de horizontal para nivelar: levantar un
brazo que no apoya no baja esa esquina, la sostiene la polea del pivote.
Mientras alguna esquina alta tenga su brazo empujando, se recoge ese antes
de bajar otro: asi, pasada la cuesta, los brazos vuelven a horizontal.

Convenios (REP-103, los del URDF): cabeceo + = morro abajo, balanceo + = lado
izquierdo arriba, elevacion + = brazo levantado. Orden {FR, FL, RR, RL}.

Modulo PURO: no importa rclpy. Probado en test/test_nivelado_core.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

N_ORUGAS = 4
# Signo de x (delante +) e y (izquierda +) de cada pivote, {FR, FL, RR, RL}
SIGNOS = ((1, -1), (1, 1), (-1, -1), (-1, 1))


@dataclass
class AjustesNivelado:
    pivote_x: float = 0.30          # m, del centro al pivote (como el URDF)
    pivote_y: float = 0.24
    ganancia: float = 1.5           # rad/s de brazo por rad de desnivel
    zona_muerta: float = math.radians(1.5)
    vel_max: float = math.radians(4.69)   # la del firmware: no sirve pedir mas
    bajada_max: float = math.radians(-60.0)  # lo mas que baja un brazo
    adelanto_max: float = math.radians(10.0)  # objetivo - medida, como mucho


def cabeceo_balanceo(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float]:
    """Cabeceo y balanceo (ZYX) de un cuaternion de orientacion."""
    s = 2.0 * (qw * qy - qz * qx)
    cabeceo = math.asin(max(-1.0, min(1.0, s)))
    balanceo = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    return cabeceo, balanceo


def desniveles(cabeceo: float, balanceo: float,
               a: AjustesNivelado = AjustesNivelado()) -> List[float]:
    """Altura de cada esquina respecto al centro, en 'radianes' (dividida por
    pivote_x). Negativo = esquina baja."""
    out = []
    for sx, sy in SIGNOS:
        z = (-sx * a.pivote_x * math.sin(cabeceo)
             + sy * a.pivote_y * math.cos(cabeceo) * math.sin(balanceo))
        out.append(z / a.pivote_x)
    return out


class Nivelador:
    """Integra los objetivos de los brazos. empezar() al activarlo y cada vez
    que otro haya movido los brazos; paso() en cada ciclo."""

    def __init__(self, ajustes: Optional[AjustesNivelado] = None) -> None:
        self.a = ajustes or AjustesNivelado()
        self.objetivo: Optional[List[float]] = None
        self._ultima = [0.0] * N_ORUGAS     # ultima medida buena de cada brazo

    @property
    def activo(self) -> bool:
        return self.objetivo is not None

    def empezar(self, medidas: Sequence[float]) -> None:
        self.objetivo = [float(q) for q in medidas]

    def parar(self) -> None:
        self.objetivo = None

    def paso(self, cabeceo: float, balanceo: float,
             medidas: Sequence[float], dt: float) -> List[float]:
        """Objetivos de los brazos, siempre finitos: el firmware descarta la
        orden ENTERA si un objetivo no lo es, y salta su watchdog."""
        a = self.a
        if self.objetivo is None:
            self.empezar(medidas)
        dt = max(0.0, min(dt, 0.2))
        inclinado = max(abs(cabeceo), abs(balanceo)) > a.zona_muerta
        des = desniveles(cabeceo, balanceo, a)
        # Primero se recogen los brazos que empujan en una esquina alta: si
        # no, tras una cuesta los brazos nunca vuelven a horizontal
        recoger = inclinado and any(
            e > 0.0 and math.isfinite(self.objetivo[i]) and self.objetivo[i] < 0.0
            for i, e in enumerate(des))
        for i, e in enumerate(des):
            q = self.objetivo[i]
            m = float(medidas[i])
            if not math.isfinite(m):
                # Sin encoder no se sabe donde esta: NaN por dentro, y fuera
                # su ultima medida (el firmware no mueve un brazo sin encoder)
                self.objetivo[i] = float('nan')
                continue
            self._ultima[i] = m
            if not math.isfinite(q):
                q = m                       # vuelve el encoder: desde la medida
            v = 0.0
            if inclinado:
                if e < 0.0 and not recoger:
                    v = a.ganancia * e                # esquina baja: bajar el brazo
                elif e > 0.0 and q < 0.0:
                    v = a.ganancia * e                # alta y empujando: recoger
            v = max(-a.vel_max, min(a.vel_max, v))
            q_nuevo = q + v * dt
            if q < 0.0 <= q_nuevo and v > 0.0:
                q_nuevo = 0.0                         # recoger, hasta horizontal
            # Que el objetivo no se escape de la medida si el brazo no llega
            q_nuevo = max(m - a.adelanto_max, min(m + a.adelanto_max, q_nuevo))
            # Nunca levantar por encima de max(q, 0) ni bajar de la cota; un
            # brazo que ya estaba por debajo de la cota no se sube ni se baja
            q_nuevo = max(min(a.bajada_max, q), min(max(q, 0.0), q_nuevo))
            self.objetivo[i] = q_nuevo
        return [q if math.isfinite(q) else self._ultima[i]
                for i, q in enumerate(self.objetivo)]
