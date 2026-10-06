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
    OPTIONS ................................ nivelado automatico (activa / desactiva)

Modulo PURO: no importa rclpy. Probado en test/test_joy_logic.py.

El esquema (pares, inclinaciones, presets, topes) vive en
starcrawler_common.orugas, compartido con el mando web.

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

from starcrawler_common.orugas import (  # noqa: F401
    FACTOR_LENTO,
    INCLINACIONES,
    MAX_ANGULAR,
    MAX_LINEAL,
    N_ORUGAS,
    PAR_DELANTERO,
    PAR_TRASERO,
    PRESETS_DEG,
    S_PRESET,
    componer_incremento,
    inclinacion,
    traccion,
)

# Tras la ultima entrada, el DS4 sigue mandando por el canal activo
S_COLA_ACTIVO = 0.2


@dataclass
class Mapeo:
    """Indices de /joy. Verificar con:  ros2 topic echo /joy

    Los valores por defecto son los de un DualShock 4 en Linux con el nodo
    `joy` (driver hid-sony): ejes 0 LX, 1 LY, 2 L2, 3 RX, 4 RY, 5 R2 y la
    cruceta en 6/7. Ese nodo niega los ejes de SDL: izquierda y arriba dan +1.
    Si algo no responde, se corrige en config/ds4.yaml sin tocar codigo.
    """
    eje_avance: int = 1
    eje_giro: int = 3
    # Los flags normalizan el eje crudo a: avance positivo = stick ARRIBA,
    # giro positivo = stick a la DERECHA. El nodo joy ya da +1 arriba y +1 a
    # la izquierda, asi que solo se invierte el giro.
    invertir_avance: bool = False
    invertir_giro: bool = True

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
    dpad_x_derecha_positivo: bool = False   # el nodo joy da +1 a la izquierda
    dpad_botones: Sequence[int] = ()

    # Deadman opcional: -1 = desactivado
    boton_enable: int = -1


@dataclass
class Ajustes:
    zona_muerta: float = 0.08
    umbral_eje: float = 0.5          # para tratar gatillos/cruceta como digital
    max_lineal: float = MAX_LINEAL   # m/s
    max_angular: float = MAX_ANGULAR  # rad/s
    factor_lento: float = FACTOR_LENTO
    s_preset: float = S_PRESET       # mantener el boton para activar la pose


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
    # DS4 tocado hace menos de S_COLA_ACTIVO: tambien va al canal activo
    activo: bool = False


def deadzone(valor: float, zona: float) -> float:
    """Igual que aplicar_deadzone() del script del PC y gc_deadzone() en C."""
    mag = abs(valor)
    if mag < zona:
        return 0.0
    signo = 1.0 if valor > 0 else -1.0
    return signo * (mag - zona) / (1.0 - zona)


class LogicaMando:
    """Convierte muestras de /joy en consignas, guardando el estado necesario
    (toggle de velocidad, temporizador de presets)."""

    def __init__(self, mapeo: Optional[Mapeo] = None,
                 ajustes: Optional[Ajustes] = None) -> None:
        self.mapeo = mapeo or Mapeo()
        self.ajustes = ajustes or Ajustes()
        self.velocidad_lenta = False
        self._l3_anterior = False
        self.nivelado = False
        self._options_anterior = False
        self._preset_pulsado = -1
        self._t_preset = 0.0
        self._posicion_vigente = False
        self._objetivo_rad = [0.0] * N_ORUGAS
        self._t_activo = -math.inf

    def cancelar_preset(self) -> None:
        """Olvida la pose enganchada; para volver a aplicarla hay que
        mantener otra vez el boton."""
        self._posicion_vigente = False
        self._preset_pulsado = -1

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
        # El modo botones se activa solo con indices validos (>= 0). Asi la
        # lista puede venir vacia O como [-1,-1,-1,-1] (el nodo ROS usa esa
        # forma porque rclpy no puede tipar un parametro con lista vacia).
        if m.dpad_botones and any(b >= 0 for b in m.dpad_botones):
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

        # Flanco de OPTIONS: se apunta siempre, se aplica tras el hombre muerto
        opt = self._boton(botones, m.boton_options)
        flanco_options = opt and not self._options_anterior
        self._options_anterior = opt

        # Parada de emergencia: manda sobre todo
        if self._boton(botones, m.boton_share):
            self.cancelar_preset()
            self.nivelado = False
            out.emergencia = True
            out.activo = self._marcar_activo(True, t)
            return out

        # Deadman opcional
        if m.boton_enable >= 0 and not self._boton(botones, m.boton_enable):
            out.activo = self._marcar_activo(False, t)
            return out

        # Nivelado (OPTIONS): al encenderlo manda el, la pose enganchada se olvida
        if flanco_options:
            self.nivelado = not self.nivelado
            if self.nivelado:
                self.cancelar_preset()

        # Traccion (siempre activa)
        av = deadzone(self._eje(ejes, m.eje_avance), a.zona_muerta)
        gi = deadzone(self._eje(ejes, m.eje_giro), a.zona_muerta)
        if m.invertir_avance:
            av = -av
        if m.invertir_giro:
            gi = -gi
        factor = a.factor_lento if self.velocidad_lenta else 1.0
        # gi + = stick a la derecha -> angular.z negativo (convencion ROS)
        out.lineal, out.angular = traccion(av, gi, factor,
                                           a.max_lineal, a.max_angular)

        # Orugas: pares + cruceta, sumados y saturados
        l1 = self._boton(botones, m.boton_l1)
        l2 = self._gatillo(ejes, botones, m.boton_l2, m.eje_l2)
        r1 = self._boton(botones, m.boton_r1)
        r2 = self._gatillo(ejes, botones, m.boton_r2, m.eje_r2)
        sentido_del = int(l1) - int(l2)
        sentido_tra = int(r1) - int(r2)
        cruceta = self._cruceta(ejes, botones)
        inclinar = self._inclinacion(*cruceta)
        out.incremento = componer_incremento(sentido_del, sentido_tra, inclinar)

        hay_manual = sentido_del != 0 or sentido_tra != 0 or any(inclinar)
        if hay_manual:
            self._posicion_vigente = False
            self.nivelado = False           # el que toca los brazos manda

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
                self.nivelado = False
                objetivo = math.radians(PRESETS_DEG[pulsado])
                self._objetivo_rad = [objetivo] * N_ORUGAS

        if self._posicion_vigente and not hay_manual:
            out.usar_posicion = True
            out.objetivo_rad = list(self._objetivo_rad)
            out.incremento = [0] * N_ORUGAS

        out.nivelar = self.nivelado

        # L3, OPTIONS y la pose ya enganchada no cuentan como tocar el mando
        tocado = (av != 0.0 or gi != 0.0 or l1 or l2 or r1 or r2
                  or any(cruceta) or pulsado >= 0)
        out.activo = self._marcar_activo(tocado, t)
        return out

    def apagar_nivelado(self) -> None:
        """Mando perdido u otra fuente al mando: el nivelado no vuelve solo."""
        self.nivelado = False

    def _marcar_activo(self, tocado: bool, t: float) -> bool:
        if tocado:
            self._t_activo = t
        return t - self._t_activo <= S_COLA_ACTIVO

    @staticmethod
    def _inclinacion(arriba: bool, abajo: bool, izq: bool, der: bool) -> List[int]:
        """Cruceta -> inclinar el conjunto (ver orugas.inclinacion). Solo con
        un sentido pulsado; en diagonal no inclina."""
        pulsados = [s for s, p in zip(INCLINACIONES, (arriba, abajo, izq, der))
                    if p]
        return inclinacion(pulsados[0] if len(pulsados) == 1 else None)
