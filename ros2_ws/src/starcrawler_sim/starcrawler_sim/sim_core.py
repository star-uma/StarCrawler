"""
sim_core.py — modelo del robot, sin ROS
=======================================
Modulo PURO: no importa rclpy ni nada de ROS, igual que joy_logic.py y
odometry_core.py. Se prueba entero en el PC.

Es el mismo modelo fisico que `starcrawler_driver/simulator.py`, pero a
nivel de CONSIGNAS en vez de a nivel de tramas serie. La diferencia
importa: cuando el ESP32 pase a micro-ROS desaparecen el protocolo serie
y el nodo driver, y con ellos aquel simulador. Este sobrevive, porque
habla el mismo idioma que hablara el firmware.

Los angulos se manejan en GRADOS DE ENCODER (180 = oruga horizontal),
que es como los lleva el firmware. La conversion a radianes de elevacion
es cosa del nodo.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

N_ORUGAS = 4

# 417 pasos/s / (400 pasos/rev x 80 de reductora) -> 4.69 deg/s
VEL_ELEVACION_DPS = 4.69

# El firmware limita 4 dps por ciclo de 10 ms -> 400 dps por segundo
RATE_LIMIT_DPS_S = 400.0

# Histeresis del control de posicion, como cc_controlPosicion
UMBRAL_PARADA_DEG = 0.5

# Bits de error, iguales a los de control_core.h
ERR_ENCODER = 0x0F
ERR_CAN = 1 << 5
ERR_WATCHDOG = 1 << 6


def rate_limit(actual: float, objetivo: float, maximo_delta: float) -> float:
    """Acerca `actual` a `objetivo` sin pasar de `maximo_delta`."""
    delta = objetivo - actual
    if delta > maximo_delta:
        return actual + maximo_delta
    if delta < -maximo_delta:
        return actual - maximo_delta
    return objetivo


class RobotSimulado:
    """Sustituye al ESP32 entero: consume consignas y produce estado."""

    def __init__(self,
                 angulos_iniciales: Optional[Sequence[float]] = None,
                 angulo_min: float = 85.0,
                 angulo_max: float = 275.0,
                 watchdog_s: float = 0.5,
                 vel_elevacion_dps: float = VEL_ELEVACION_DPS,
                 encoders_ok: bool = True,
                 can_ok: bool = True) -> None:
        self.angulo: List[float] = list(angulos_iniciales or [180.0] * N_ORUGAS)
        self.angulo_min = angulo_min
        self.angulo_max = angulo_max
        self.watchdog_s = watchdog_s
        self.vel_elevacion_dps = vel_elevacion_dps
        self.encoders_ok = encoders_ok
        self.can_ok = can_ok

        self.vel_izq_dps = 0.0
        self.vel_der_dps = 0.0
        self.seguridad = True

        self._obj_izq_dps = 0.0
        self._obj_der_dps = 0.0
        self._incremento = [0] * N_ORUGAS
        self._objetivo_deg = [180.0] * N_ORUGAS
        self._usar_posicion = False
        self._emergencia = False

        self._t = 0.0
        self._t_ultimo_cmd = -1e9

    # --- Entradas (lo que llegaria por los topicos) ----------------------

    def consigna_traccion(self, vel_izq_dps: float, vel_der_dps: float) -> None:
        self._obj_izq_dps = vel_izq_dps
        self._obj_der_dps = vel_der_dps
        self._t_ultimo_cmd = self._t

    def consigna_orugas(self,
                        incrementos: Sequence[int],
                        objetivos_deg: Sequence[float],
                        usar_posicion: bool,
                        emergencia: bool) -> None:
        self._incremento = list(incrementos)
        self._objetivo_deg = list(objetivos_deg)
        self._usar_posicion = usar_posicion
        self._emergencia = emergencia
        self._t_ultimo_cmd = self._t

    # --- Integracion -----------------------------------------------------

    def avanzar(self, dt: float) -> None:
        """Integra el modelo dt segundos."""
        if dt <= 0.0:
            return
        self._t += dt

        # Watchdog: sin consignas frescas, estado seguro. Es la misma
        # proteccion que tiene el firmware, y conviene que el simulador la
        # tenga para que un fallo del grafo se note tambien aqui.
        self.seguridad = (self._t - self._t_ultimo_cmd) > self.watchdog_s

        if self.seguridad or self._emergencia:
            self.vel_izq_dps = 0.0
            self.vel_der_dps = 0.0
            return

        maximo = RATE_LIMIT_DPS_S * dt
        self.vel_izq_dps = rate_limit(self.vel_izq_dps, self._obj_izq_dps, maximo)
        self.vel_der_dps = rate_limit(self.vel_der_dps, self._obj_der_dps, maximo)

        self._mover_orugas(dt)

    def _mover_orugas(self, dt: float) -> None:
        paso = self.vel_elevacion_dps * dt
        for i in range(N_ORUGAS):
            if self._usar_posicion:
                # Sin encoder valido no hay lazo cerrado posible: el
                # firmware inhibe ese brazo y aqui se replica.
                if not self.encoders_ok:
                    continue
                error = self._objetivo_deg[i] - self.angulo[i]
                if abs(error) <= UMBRAL_PARADA_DEG:
                    continue
                sentido = 1.0 if error > 0 else -1.0
                self.angulo[i] += sentido * min(paso, abs(error))
            else:
                inc = self._incremento[i]
                if inc:
                    self.angulo[i] += (1.0 if inc > 0 else -1.0) * paso

            self.angulo[i] = max(self.angulo_min,
                                 min(self.angulo_max, self.angulo[i]))

    # --- Salida (lo que se publicaria) -----------------------------------

    @property
    def bits_error(self) -> int:
        errores = 0
        if not self.encoders_ok:
            errores |= ERR_ENCODER
        if not self.can_ok:
            errores |= ERR_CAN
        if self.seguridad:
            errores |= ERR_WATCHDOG
        return errores

    def estado(self) -> Tuple[List[float], float, float, bool, int]:
        """(angulos_deg, vel_izq_dps, vel_der_dps, seguridad, bits_error)"""
        return (list(self.angulo), self.vel_izq_dps, self.vel_der_dps,
                self.seguridad, self.bits_error)


# --- Conversion de angulos -----------------------------------------------

# OJO: esta conversion vive tambien en el firmware de micro-ROS (app.c) y
# en starcrawler_gui. Son tres copias de la misma regla; si cambia el
# convenio hay que tocarlas todas.

def es_espejada(i: int) -> bool:
    """FL y RR van espejadas, como documenta el TFG."""
    return i == 1 or i == 2


def elevacion_a_encoder_deg(i: int, elevacion_rad: float) -> float:
    """Radianes de elevacion (+ = brazo levantado) -> grados de encoder."""
    d = elevacion_rad * 180.0 / math.pi
    return (180.0 + d) if es_espejada(i) else (180.0 - d)


def encoder_a_elevacion_rad(i: int, encoder_deg: float) -> float:
    """Grados de encoder -> radianes de elevacion."""
    d = (encoder_deg - 180.0) if es_espejada(i) else (180.0 - encoder_deg)
    return d * math.pi / 180.0
