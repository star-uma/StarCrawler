"""
sim_core.py — modelo del robot, sin ROS
=======================================
Modulo PURO: no importa rclpy ni nada de ROS, igual que joy_logic.py y
odometry_core.py. Se prueba entero en el PC.

Imita al ESP32 con la app de micro-ROS (micro_ros_esp32_apps/starcrawler_app):
el lazo de control a 100 Hz de app.c, las reglas de control_core.c y la ISR
de los steppers de hw.c, con su rampa. Los numeros son los de su config.h;
si cambian alli, cambian aqui.

Los angulos se manejan en GRADOS DE ENCODER (180 = oruga horizontal), que es
como los lleva el firmware. La conversion a radianes de elevacion es cosa
del nodo.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

# La traduccion de angulos vive en starcrawler_common: estaba
# copiada aqui y en la GUI, y un error de signo ahi mueve los
# brazos al reves.
from starcrawler_common.angulos import (  # noqa: F401
    es_espejada,
    elevacion_a_encoder_deg,
    encoder_a_elevacion_rad,
)

N_ORUGAS = 4

# --- config.h de la app de micro-ROS -----------------------------------------

CICLO_CONTROL_S = 0.010             # CICLO_CONTROL_MS
WATCHDOG_S = 0.300                  # WATCHDOG_TIMEOUT_MS
VEL_MAX_DPS = 40.0
RATE_LIMIT_DPS_CICLO = 4.0
UMBRAL_ARRANQUE_DEG = 1.0
UMBRAL_PARADA_DEG = 0.5
ANGULO_MIN_DEG = 85.0
ANGULO_MAX_DEG = 275.0
OFFSETS_ENCODER = (-2.0, -5.0, 16.0, -15.0)

# ISR de los steppers: tick de 50 us y semiperiodos en ticks, con rampa
TICK_ISR_S = 50e-6
SEMI_ARRANQUE_TICKS = 4800 // 50    # SEMIPERIODO_ARRANQUE_US
SEMI_REGIMEN_TICKS = 1200 // 50     # SEMIPERIODO_STEP_US
RAMPA_DECREMENTO_TICKS = 2

# 400 pulsos por vuelta en el DM542 y reductora 1:80
GRADOS_POR_PULSO = 360.0 / (400 * 80)
VEL_ELEVACION_DPS = GRADOS_POR_PULSO / (2 * SEMI_REGIMEN_TICKS * TICK_ISR_S)

# AS5600: 12 bits por vuelta
GRADOS_POR_CUENTA = 360.0 / 4096

# Bits de error, iguales a los del firmware
ERR_ENCODER = 0x0F
ERR_CAN = 1 << 5
ERR_WATCHDOG = 1 << 6
ERR_SIMULADO = 1 << 7


def rate_limit(actual: float, objetivo: float, maximo_delta: float) -> float:
    """Acerca `actual` a `objetivo` sin pasar de `maximo_delta`."""
    delta = objetivo - actual
    if delta > maximo_delta:
        return actual + maximo_delta
    if delta < -maximo_delta:
        return actual - maximo_delta
    return objetivo


def saturar(valor: float, maximo: float) -> float:
    return max(-maximo, min(maximo, valor))


def control_posicion(actual: float, objetivo: float, en_marcha: bool) -> int:
    """cc_controlPosicion: arranca por encima de 1 grado y para por debajo de 0,5."""
    error = objetivo - actual
    umbral = UMBRAL_PARADA_DEG if en_marcha else UMBRAL_ARRANQUE_DEG
    if abs(error) <= umbral:
        return 0
    return 1 if error > 0 else -1


def aplicar_limites(cmd: int, angulo: float, encoder_ok: bool,
                    minimo: float, maximo: float) -> int:
    """cc_aplicarLimites: sin encoder no hay limites."""
    if not encoder_ok:
        return cmd
    if cmd > 0 and angulo >= maximo:
        return 0
    if cmd < 0 and angulo <= minimo:
        return 0
    return cmd


def leer_as5600(angulo: float, offset: float) -> float:
    """Lo que devolveria el encoder: cuenta entera de 12 bits mas el offset."""
    cuenta = round((angulo - offset) / GRADOS_POR_CUENTA) % 4096
    return cuenta * GRADOS_POR_CUENTA + offset


class _Stepper:
    """Un canal de la ISR: semiperiodo con rampa y un paso por flanco de subida."""

    def __init__(self) -> None:
        self.sentido = 0
        self.semi = SEMI_ARRANQUE_TICKS
        self.fase = 0

    def comando(self, cmd: int) -> None:
        if cmd == 0:
            self.sentido = 0
            self.fase = 0
            return
        sentido = 1 if cmd > 0 else -1
        if sentido != self.sentido:     # arranque o inversion: rampa desde cero
            self.sentido = sentido
            self.semi = SEMI_ARRANQUE_TICKS
            self.fase = 0

    def pulsos(self, ticks: int) -> int:
        if self.sentido == 0:
            return 0
        n = 0
        while ticks > 0:
            if self.fase < self.semi:
                paso = min(ticks, self.semi - self.fase)
                self.fase += paso
                ticks -= paso
                if self.fase == self.semi:
                    n += 1
            else:
                paso = min(ticks, 2 * self.semi - self.fase)
                self.fase += paso
                ticks -= paso
                if self.fase == 2 * self.semi:
                    self.fase = 0
                    self.semi = max(SEMI_REGIMEN_TICKS,
                                    self.semi - RAMPA_DECREMENTO_TICKS)
        return n * self.sentido


class RobotSimulado:
    """Sustituye al ESP32 entero: consume consignas y produce estado."""

    def __init__(self,
                 angulos_iniciales: Optional[Sequence[float]] = None,
                 angulo_min: float = ANGULO_MIN_DEG,
                 angulo_max: float = ANGULO_MAX_DEG,
                 watchdog_s: float = WATCHDOG_S,
                 vel_max_dps: float = VEL_MAX_DPS,
                 encoders_ok: bool = True,
                 can_ok: bool = True) -> None:
        # Angulo real de cada brazo y el ultimo que dio su encoder
        self.angulo: List[float] = list(angulos_iniciales or [180.0] * N_ORUGAS)
        self._medido: List[float] = list(self.angulo)
        self.angulo_min = angulo_min
        self.angulo_max = angulo_max
        self.watchdog_s = watchdog_s
        self.vel_max_dps = vel_max_dps
        self.encoders_ok = encoders_ok
        self.can_ok = can_ok

        self.vel_izq_dps = 0.0
        self.vel_der_dps = 0.0
        # Como enSeguridad en app.c: watchdog vencido o emergencia
        self.seguridad = True
        self._vencido = True

        self._obj_izq_dps = 0.0
        self._obj_der_dps = 0.0
        self._incremento = [0] * N_ORUGAS
        self._objetivo_deg = [180.0] * N_ORUGAS
        self._usar_posicion = False
        self._emergencia = False

        self._steppers = [_Stepper() for _ in range(N_ORUGAS)]
        self._en_marcha = [False] * N_ORUGAS

        self._t = 0.0
        self._t_ultimo_cmd = -1e9
        self._resto_s = 0.0

    # --- Entradas (lo que llegaria por los topicos) ----------------------

    def consigna_traccion(self, vel_izq_dps: float, vel_der_dps: float) -> None:
        # cb_cmd_vel satura cada lado antes de la rampa
        self._obj_izq_dps = saturar(vel_izq_dps, self.vel_max_dps)
        self._obj_der_dps = saturar(vel_der_dps, self.vel_max_dps)
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
        """Integra el modelo dt segundos, en ciclos de control de 10 ms."""
        if dt <= 0.0:
            return
        self._resto_s += dt
        while self._resto_s >= CICLO_CONTROL_S - 1e-9:
            self._resto_s -= CICLO_CONTROL_S
            self._ciclo()

    def _ciclo(self) -> None:
        self._t += CICLO_CONTROL_S

        if self.encoders_ok:
            self._medido = [leer_as5600(self.angulo[i], OFFSETS_ENCODER[i])
                            for i in range(N_ORUGAS)]

        self._vencido = (self._t - self._t_ultimo_cmd) > self.watchdog_s
        self.seguridad = self._vencido or self._emergencia
        if self.seguridad:
            self.vel_izq_dps = 0.0
            self.vel_der_dps = 0.0
            for i in range(N_ORUGAS):
                self._steppers[i].comando(0)
                self._en_marcha[i] = False
            return

        for i in range(N_ORUGAS):
            if self._usar_posicion:
                cmd = (control_posicion(self._medido[i], self._objetivo_deg[i],
                                        self._en_marcha[i])
                       if self.encoders_ok else 0)
            else:
                inc = self._incremento[i]
                cmd = (inc > 0) - (inc < 0)
            cmd = aplicar_limites(cmd, self._medido[i], self.encoders_ok,
                                  self.angulo_min, self.angulo_max)
            self._en_marcha[i] = cmd != 0
            self._steppers[i].comando(cmd)

        self.vel_izq_dps = rate_limit(self.vel_izq_dps, self._obj_izq_dps,
                                      RATE_LIMIT_DPS_CICLO)
        self.vel_der_dps = rate_limit(self.vel_der_dps, self._obj_der_dps,
                                      RATE_LIMIT_DPS_CICLO)

        # Lo que la ISR alcanza a dar hasta el siguiente ciclo
        ticks = round(CICLO_CONTROL_S / TICK_ISR_S)
        for i in range(N_ORUGAS):
            self.angulo[i] += self._steppers[i].pulsos(ticks) * GRADOS_POR_PULSO

    # --- Salida (lo que se publicaria) -----------------------------------

    @property
    def angulo_medido(self) -> List[float]:
        """Lo que publica el firmware: con el encoder caido, el ultimo bueno."""
        return list(self._medido)

    @property
    def bits_error(self) -> int:
        errores = ERR_SIMULADO
        if not self.encoders_ok:
            errores |= ERR_ENCODER
        if not self.can_ok:
            errores |= ERR_CAN
        if self._vencido:
            errores |= ERR_WATCHDOG
        return errores

    def estado(self) -> Tuple[List[float], float, float, bool, int]:
        """(angulos_deg medidos, vel_izq_dps, vel_der_dps, seguridad, bits_error)"""
        return (self.angulo_medido, self.vel_izq_dps, self.vel_der_dps,
                self.seguridad, self.bits_error)
