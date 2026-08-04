"""
joy_logic.py — logica del mando (esquema simultaneo)
====================================================
Puerto a Python del esquema SIMULTANEO de gamepad_core (firmware standalone):
la traccion esta SIEMPRE activa y las orugas se mueven superpuestas.

    Stick izq. vertical / der. horizontal .. avanzar / girar (siempre)
    L1 / L2 ................................ par DELANTERO sube / baja
    R1 / R2 ................................ par TRASERO   sube / baja
    Cruceta ................................ inclinar el conjunto
    X / O / [] / T (mantener) .............. presets de pose
    SHARE .................................. parada de emergencia
    L3 ..................................... velocidad lenta / rapida
    OPTIONS ................................ (reservado: nivelado con IMU)

Modulo PURO: no importa rclpy. Probado en test/test_joy_logic.py.

Nota sobre los presets: en el TFG las poses se daban en grados de encoder
(225/180/135/90) y habia que espejar FL y RR. Aqui se trabaja en ELEVACION
(positivo = brazo levantado), donde la pose es simetrica y las cuatro orugas
comparten el mismo valor: -45 / 0 / +45 / +90 grados. El espejado lo hace el
driver al bajar a grados de encoder.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

N_ORUGAS = 4

# Comando incremental para SUBIR cada par (en elevacion, +1 = subir)
PAR_DELANTERO = (1, 1, 0, 0)   # FR, FL
PAR_TRASERO = (0, 0, 1, 1)     # RR, RL

# Presets de pose, en grados de ELEVACION (equivalen a 225/180/135/90 de
# encoder en la referencia F.R. del TFG)
PRESETS_DEG = (-45.0, 0.0, 45.0, 90.0)


@dataclass
class Mapeo:
    """Indices de /joy. Verificar con:  ros2 topic echo /joy

    Los valores por defecto son los tipicos de un DualShock 4 en Linux con el
    nodo `joy`. Si algo no responde, se corrige en config/ds4.yaml sin tocar
    codigo.
    """
    eje_avance: int = 1
    eje_giro: int = 2
    # Los flags normalizan el eje crudo a: avance positivo = stick ARRIBA,
    # giro positivo = stick a la DERECHA. En Linux el eje Y da -1 arriba (de
    # ahi el true) y el X da +1 a la derecha (de ahi el false).
    invertir_avance: bool = True
    invertir_giro: bool = False

    boton_l1: int = 4
    boton_l2: int = 6
    boton_r1: int = 5
    boton_r2: int = 7
    # Algunos drivers exponen L2/R2 solo como ejes analogicos (-1 suelto,
    # +1 a fondo). Si es tu caso, poner aqui su indice; -1 = no usar.
    eje_l2: int = -1
    eje_r2: int = -1
    boton_l3: int = 11
    boton_share: int = 8
    boton_options: int = 9
    botones_preset: Sequence[int] = (0, 1, 3, 2)   # X, O, [], T

    # Cruceta: por defecto como ejes (hat). Si en tu mando son botones,
    # poner dpad_botones = [arriba, abajo, izq, der] y dpad_eje_x = -1.
    dpad_eje_x: int = 6
    dpad_eje_y: int = 7
    dpad_y_arriba_positivo: bool = True
    dpad_x_derecha_positivo: bool = True
    dpad_botones: Sequence[int] = ()

    # Deadman opcional: -1 = desactivado
    boton_enable: int = -1


@dataclass
class Ajustes:
    zona_muerta: float = 0.08
    umbral_eje: float = 0.5          # para tratar gatillos/cruceta como digital
    max_lineal: float = 0.017        # m/s  (40 dps * r=0.025 m)
    max_angular: float = 0.087       # rad/s
    factor_lento: float = 0.5
    s_preset: float = 0.5            # mantener el boton para activar la pose


@dataclass
class Salida:
    lineal: float = 0.0
    angular: float = 0.0
    incremento: List[int] = field(default_factory=lambda: [0] * N_ORUGAS)
    usar_posicion: bool = False
    objetivo_rad: List[float] = field(default_factory=lambda: [0.0] * N_ORUGAS)
    emergencia: bool = False
    velocidad_lenta: bool = False
    nivelar: bool = False


def deadzone(valor: float, zona: float) -> float:
    """Igual que aplicar_deadzone() del script del PC y gc_deadzone() en C."""
    mag = abs(valor)
    if mag < zona:
        return 0.0
    signo = 1.0 if valor > 0 else -1.0
    return signo * (mag - zona) / (1.0 - zona)


def _signo(v: int) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)


class LogicaMando:
    """Convierte muestras de /joy en consignas, guardando el estado necesario
    (toggle de velocidad, temporizador de presets)."""

    def __init__(self, mapeo: Optional[Mapeo] = None,
                 ajustes: Optional[Ajustes] = None) -> None:
        self.mapeo = mapeo or Mapeo()
        self.ajustes = ajustes or Ajustes()
        self.velocidad_lenta = False
        self._l3_anterior = False
        self._preset_pulsado = -1
        self._t_preset = 0.0
        self._posicion_vigente = False
        self._objetivo_rad = [0.0] * N_ORUGAS

    # ─── Lectura del mensaje Joy ─────────────────────────────────────────

    def _eje(self, ejes: Sequence[float], idx: int) -> float:
        return ejes[idx] if 0 <= idx < len(ejes) else 0.0

    def _boton(self, botones: Sequence[int], idx: int) -> bool:
        return bool(botones[idx]) if 0 <= idx < len(botones) else False

    def _gatillo(self, ejes: Sequence[float], botones: Sequence[int],
                 idx_boton: int, idx_eje: int) -> bool:
        """Un gatillo puede llegar como boton, como eje analogico, o ambos."""
        if self._boton(botones, idx_boton):
            return True
        if 0 <= idx_eje < len(ejes):
            return ejes[idx_eje] > 0.0   # reposo -1, a fondo +1
        return False

    def _cruceta(self, ejes: Sequence[float],
                 botones: Sequence[int]) -> tuple:
        m = self.mapeo
        if m.dpad_botones:
            b = list(m.dpad_botones) + [-1] * 4
            return (self._boton(botones, b[0]), self._boton(botones, b[1]),
                    self._boton(botones, b[2]), self._boton(botones, b[3]))
        u = self.ajustes.umbral_eje
        x = self._eje(ejes, m.dpad_eje_x)
        y = self._eje(ejes, m.dpad_eje_y)
        if not m.dpad_y_arriba_positivo:
            y = -y
        if not m.dpad_x_derecha_positivo:
            x = -x
        return (y > u, y < -u, x < -u, x > u)

    # ─── Procesado ───────────────────────────────────────────────────────

    def procesar(self, ejes: Sequence[float], botones: Sequence[int],
                 t: float) -> Salida:
        m, a = self.mapeo, self.ajustes
        out = Salida()

        # Toggle de velocidad lenta (flanco de L3)
        l3 = self._boton(botones, m.boton_l3)
        if l3 and not self._l3_anterior:
            self.velocidad_lenta = not self.velocidad_lenta
        self._l3_anterior = l3
        out.velocidad_lenta = self.velocidad_lenta

        # Parada de emergencia: manda sobre todo
        if self._boton(botones, m.boton_share):
            self._posicion_vigente = False
            self._preset_pulsado = -1
            out.emergencia = True
            return out

        # Deadman opcional
        if m.boton_enable >= 0 and not self._boton(botones, m.boton_enable):
            return out

        # Traccion (siempre activa)
        av = deadzone(self._eje(ejes, m.eje_avance), a.zona_muerta)
        gi = deadzone(self._eje(ejes, m.eje_giro), a.zona_muerta)
        if m.invertir_avance:
            av = -av
        if m.invertir_giro:
            gi = -gi
        factor = a.factor_lento if self.velocidad_lenta else 1.0
        out.lineal = av * a.max_lineal * factor
        # Convencion ROS: angular.z positivo = giro a la IZQUIERDA. 'gi' ya
        # esta normalizado a positivo = stick a la derecha, de ahi el signo.
        out.angular = -gi * a.max_angular * factor

        # Orugas: pares + cruceta, sumados y saturados
        manual = [0] * N_ORUGAS
        sentido_del = ((1 if self._boton(botones, m.boton_l1) else 0)
                       - (1 if self._gatillo(ejes, botones, m.boton_l2, m.eje_l2) else 0))
        sentido_tra = ((1 if self._boton(botones, m.boton_r1) else 0)
                       - (1 if self._gatillo(ejes, botones, m.boton_r2, m.eje_r2) else 0))
        for i in range(N_ORUGAS):
            manual[i] += sentido_del * PAR_DELANTERO[i]
            manual[i] += sentido_tra * PAR_TRASERO[i]

        arriba, abajo, izq, der = self._cruceta(ejes, botones)
        inclinacion = self._inclinacion(arriba, abajo, izq, der)
        hay_manual = sentido_del != 0 or sentido_tra != 0
        for i in range(N_ORUGAS):
            manual[i] += inclinacion[i]
            if inclinacion[i] != 0:
                hay_manual = True
            out.incremento[i] = _signo(manual[i])

        if hay_manual:
            self._posicion_vigente = False

        # Presets de pose: mantener el boton s_preset segundos
        pulsado = -1
        for k, idx in enumerate(m.botones_preset):
            if self._boton(botones, idx):
                pulsado = k
                break
        if pulsado < 0:
            self._preset_pulsado = -1
        else:
            if self._preset_pulsado != pulsado:
                self._preset_pulsado = pulsado
                self._t_preset = t
            elif t - self._t_preset >= a.s_preset:
                self._posicion_vigente = True
                objetivo = math.radians(PRESETS_DEG[pulsado])
                self._objetivo_rad = [objetivo] * N_ORUGAS

        if self._posicion_vigente and not hay_manual:
            out.usar_posicion = True
            out.objetivo_rad = list(self._objetivo_rad)
            out.incremento = [0] * N_ORUGAS

        out.nivelar = self._boton(botones, m.boton_options)
        return out

    @staticmethod
    def _inclinacion(arriba: bool, abajo: bool, izq: bool, der: bool) -> List[int]:
        """Cruceta -> inclinar el conjunto. Es el modo 3 del TFG traducido a
        ELEVACION (+1 = ese brazo sube). Al subir los brazos de un extremo, ese
        extremo del chasis pierde apoyo y baja.

        Equivalencias con los vectores en grados de encoder del TFG:
            arriba {-1,1,-1,1} -> {+1,+1,-1,-1}   abajo  { 1,-1, 1,-1} -> {-1,-1,+1,+1}
            izq    { 1,1,-1,-1} -> {-1,+1,-1,+1}   der    {-1,-1, 1, 1} -> {+1,-1,+1,-1}
        """
        if arriba and not (abajo or izq or der):
            return [1, 1, -1, -1]     # suben delanteras -> inclinar adelante
        if abajo and not (arriba or izq or der):
            return [-1, -1, 1, 1]     # suben traseras   -> inclinar atras
        if izq and not (arriba or abajo or der):
            return [-1, 1, -1, 1]     # suben las del lado izquierdo
        if der and not (arriba or abajo or izq):
            return [1, -1, 1, -1]     # suben las del lado derecho
        return [0, 0, 0, 0]
