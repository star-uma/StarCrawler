"""
simulator.py — ESP32 simulado
=============================
Sustituto del puerto serie con la misma interfaz que pyserial (write / read /
in_waiting / close), para poder levantar TODO el grafo ROS 2 y ver el robot
moverse en RViz sin hardware:

    ros2 launch starcrawler_bringup robot.launch.py simulate:=true

Modelo: integra el angulo de cada oruga a la velocidad real de los steppers
(400 pasos/rev x reductora 1:80, ~417 pasos/s -> ~4.69 deg/s), respeta los
limites de recorrido y responde con tramas STATE a 50 Hz.

Modulo PURO (no importa rclpy ni pyserial): probado en test/test_simulator.py.
"""
from __future__ import annotations

from typing import List

from . import protocol as proto

# 417 pasos/s / (400 pasos/rev * 80) = 0.01303 rev/s = 4.69 deg/s
VEL_ELEVACION_DPS = 4.69


class Esp32Simulado:
    """Modelo minimo del firmware: consume tramas CMD y emite tramas STATE."""

    def __init__(self,
                 angulos_iniciales: List[float] | None = None,
                 periodo_estado_s: float = 0.02,
                 watchdog_s: float = 0.3,
                 angulo_min: float = 85.0,
                 angulo_max: float = 275.0,
                 can_ok: bool = True,
                 encoders_ok: bool = True) -> None:
        self.angulo = list(angulos_iniciales or [180.0] * proto.N_ORUGAS)
        self.periodo_estado_s = periodo_estado_s
        self.watchdog_s = watchdog_s
        self.angulo_min = angulo_min
        self.angulo_max = angulo_max
        self.can_ok = can_ok
        self.encoders_ok = encoders_ok

        self.vel_izq_dps = 0.0
        self.vel_der_dps = 0.0
        self.seguridad = True

        self._parser = proto.ParserComando()
        self._cmd = proto.Comando()
        self._salida = bytearray()
        self._t = 0.0
        self._t_ultimo_cmd = -1e9
        self._t_ultimo_estado = -1e9

    # ─── Interfaz tipo pyserial ──────────────────────────────────────────

    @property
    def in_waiting(self) -> int:
        return len(self._salida)

    def write(self, datos: bytes) -> int:
        for cmd in self._parser.alimentar(bytes(datos)):
            self._cmd = cmd
            self._t_ultimo_cmd = self._t
            self.seguridad = False
        return len(datos)

    def read(self, n: int = 1) -> bytes:
        trozo = bytes(self._salida[:n])
        del self._salida[:n]
        return trozo

    def close(self) -> None:
        pass

    @property
    def is_open(self) -> bool:
        return True

    # ─── Modelo ──────────────────────────────────────────────────────────

    def avanzar(self, dt: float) -> None:
        """Integra el modelo dt segundos y encola tramas STATE si toca."""
        self._t += dt

        if self._t - self._t_ultimo_cmd > self.watchdog_s:
            self.seguridad = True

        if self.seguridad or self._cmd.emergencia:
            self.vel_izq_dps = 0.0
            self.vel_der_dps = 0.0
        else:
            self.vel_izq_dps = self._cmd.vel_izq_cdps / 100.0
            self.vel_der_dps = self._cmd.vel_der_cdps / 100.0
            self._mover_orugas(dt)

        if self._t - self._t_ultimo_estado >= self.periodo_estado_s:
            self._t_ultimo_estado = self._t
            self._salida.extend(proto.empaquetar_estado(self._estado()))

    def _mover_orugas(self, dt: float) -> None:
        paso = VEL_ELEVACION_DPS * dt
        for i in range(proto.N_ORUGAS):
            if self._cmd.usar_posicion:
                if not self.encoders_ok:
                    continue  # sin realimentacion no hay lazo cerrado
                objetivo = self._cmd.objetivo_cdeg[i] / 100.0
                error = objetivo - self.angulo[i]
                if abs(error) <= 0.5:
                    continue
                sentido = 1.0 if error > 0 else -1.0
                self.angulo[i] += sentido * min(paso, abs(error))
            else:
                inc = self._cmd.incremento[i]
                if inc:
                    self.angulo[i] += (1.0 if inc > 0 else -1.0) * paso
            # limites software del firmware
            self.angulo[i] = max(self.angulo_min, min(self.angulo_max, self.angulo[i]))

    def _estado(self) -> proto.Estado:
        errores = 0
        if not self.encoders_ok:
            errores |= proto.ERR_ENCODER
        if not self.can_ok:
            errores |= proto.ERR_CAN
        if self.seguridad:
            errores |= proto.ERR_WATCHDOG
        return proto.Estado(
            angulo_cdeg=[int(round(a * 100)) for a in self.angulo],
            vel_izq_cdps=int(round(self.vel_izq_dps * 100)),
            vel_der_cdps=int(round(self.vel_der_dps * 100)),
            error_bits=errores,
            seguridad=self.seguridad,
            imu_ok=False,
            seq_eco=self._cmd.seq,
        )
