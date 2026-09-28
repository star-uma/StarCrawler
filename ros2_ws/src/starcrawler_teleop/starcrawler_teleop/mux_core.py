"""
mux_core.py — multiplexor de /crawler/command, sin ROS
======================================================
Lo que twist_mux hace con /cmd_vel, para las orugas y con dos reglas mas:

- OR de las emergencias: si una fuente fresca pide parada, sale parada,
  aunque mande otra de mas prioridad;
- enganche: la emergencia de una fuente con `engancha` se mantiene hasta
  rearmar a proposito, aunque la fuente se calle.

Una fuente es fresca si su ultimo mensaje LLEGO hace `timeout_s` o menos,
medido con el reloj del nodo (nunca con header.stamp).

Modulo PURO: no importa rclpy. Probado en test/test_mux_core.py.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

N_ORUGAS = 4

EMERGENCIA_ACTIVA = 'emergencia'   # valor de crawler_mux/activa

CLAVES_FUENTE = ('topic', 'timeout', 'priority', 'engancha')


@dataclass(frozen=True)
class Fuente:
    nombre: str
    prioridad: int
    timeout_s: float
    engancha: bool = False


@dataclass(frozen=True)
class Orden:
    incremento: Tuple[int, ...] = (0,) * N_ORUGAS
    usar_posicion: bool = False
    objetivo_rad: Tuple[float, ...] = (0.0,) * N_ORUGAS
    emergencia: bool = False


EMERGENCIA = Orden(emergencia=True)


class Mux:

    def __init__(self, fuentes: Iterable[Fuente]) -> None:
        self._fuentes: Dict[str, Fuente] = {}
        for f in fuentes:
            if f.nombre in self._fuentes or f.nombre in ('', EMERGENCIA_ACTIVA):
                raise ValueError(f'nombre de fuente no valido: {f.nombre!r}')
            if f.timeout_s <= 0:
                raise ValueError(f'{f.nombre}: el timeout tiene que ser > 0')
            self._fuentes[f.nombre] = f
        if not self._fuentes:
            raise ValueError('no hay fuentes')
        prioridades = [f.prioridad for f in self._fuentes.values()]
        if len(set(prioridades)) != len(prioridades):
            raise ValueError('dos fuentes con la misma prioridad')
        self._ultima: Dict[str, Tuple[Orden, float]] = {}
        self.enganchado = False

    @property
    def fuentes(self) -> List[Fuente]:
        return list(self._fuentes.values())

    def _fresca(self, nombre: str, t: float) -> bool:
        if nombre not in self._ultima:
            return False
        edad = t - self._ultima[nombre][1]
        # edad < 0: el reloj ha ido hacia atras; mejor caducar que eternizar
        return 0.0 <= edad <= self._fuentes[nombre].timeout_s

    def _mas_prioritaria(self, t: float) -> Optional[Fuente]:
        frescas = [f for f in self._fuentes.values() if self._fresca(f.nombre, t)]
        return max(frescas, key=lambda f: f.prioridad) if frescas else None

    def piden_emergencia(self, t: float) -> List[str]:
        """Fuentes frescas cuyo ultimo mensaje pide emergencia."""
        return [n for n, (orden, _) in self._ultima.items()
                if orden.emergencia and self._fresca(n, t)]

    def recibir(self, nombre: str, orden: Orden, t: float) -> Optional[Orden]:
        """Orden a publicar al llegar este mensaje, o None."""
        fuente = self._fuentes[nombre]
        self._ultima[nombre] = (orden, t)
        if self.enganchado:
            return None
        if orden.emergencia:
            if fuente.engancha:
                self.enganchado = True
            return EMERGENCIA
        if self._mas_prioritaria(t) is not fuente:
            return None
        return EMERGENCIA if self.piden_emergencia(t) else orden

    def tick(self, t: float) -> Optional[Orden]:
        return EMERGENCIA if self.enganchado else None

    def rearmar(self, t: float) -> bool:
        """Quita el enganche salvo que una fuente fresca pida emergencia."""
        if self.piden_emergencia(t):
            return False
        self.enganchado = False
        return True

    def activa(self, t: float) -> str:
        if self.enganchado:
            return EMERGENCIA_ACTIVA
        f = self._mas_prioritaria(t)
        return f.nombre if f else ''


def _numero(v: object) -> bool:
    return type(v) in (int, float)


def leer_fuentes(parametros: Mapping[str, object]) -> List[Tuple[Fuente, str]]:
    """Parametros bajo `topics` ({'web.topic': ..., 'web.timeout': ...}) ->
    [(Fuente, topico)], en el orden en que aparecen."""
    tablas: Dict[str, Dict[str, object]] = {}
    for clave, valor in parametros.items():
        nombre, _, campo = clave.rpartition('.')
        if not nombre or '.' in nombre:
            raise ValueError(f'topics.{clave}: se esperaba topics.<fuente>.<campo>')
        if campo not in CLAVES_FUENTE:
            raise ValueError(f'topics.{clave}: campo desconocido')
        tablas.setdefault(nombre, {})[campo] = valor

    fuentes = []
    for nombre, tabla in tablas.items():
        for campo in ('topic', 'timeout', 'priority'):
            if campo not in tabla:
                raise ValueError(f'topics.{nombre}: falta {campo}')
        topico = tabla['topic']
        timeout = tabla['timeout']
        prioridad = tabla['priority']
        engancha = tabla.get('engancha', False)
        if type(topico) is not str or not topico:
            raise ValueError(f'topics.{nombre}.topic: tiene que ser un texto')
        if not _numero(timeout):
            raise ValueError(f'topics.{nombre}.timeout: tiene que ser un numero')
        if type(prioridad) is not int:
            raise ValueError(f'topics.{nombre}.priority: tiene que ser entero')
        if type(engancha) is not bool:
            raise ValueError(f'topics.{nombre}.engancha: tiene que ser bool')
        fuentes.append((Fuente(nombre, prioridad, float(timeout), engancha),
                        topico))
    return fuentes
