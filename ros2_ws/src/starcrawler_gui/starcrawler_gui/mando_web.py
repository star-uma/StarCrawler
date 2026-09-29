"""
mando_web.py — el mando de la pagina /3d, sin ROS
=================================================
Lo que llega por POST se comprueba aqui y lo arbitra Arbitro: manda un solo
puesto a la vez, cada orden vale 0,2 s y los presets se aplican como en el
DS4 (mantener s_preset). El esquema de las orugas y los signos de la
traccion son los de starcrawler_common.orugas, los mismos del teleop.

Modulo PURO: no importa rclpy. Probado en test/test_mando_web.py.
servidor.py lo llama desde sus hilos HTTP y gui_node publica lo que
devuelve salida() desde su temporizador. Diseno en la issue #18.
"""
from __future__ import annotations

import hmac
import json
import math
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from starcrawler_common.orugas import (
    FACTOR_LENTO,
    INCLINACIONES,
    MAX_ANGULAR,
    MAX_LINEAL,
    N_ORUGAS,
    PRESETS_DEG,
    S_PRESET,
    componer_incremento,
    inclinacion,
    traccion,
)

FRESCURA_S = 0.2
SUELTA_DUENO_S = 1.0
VENTANA_EMERGENCIA_S = 0.2
# Una orden que tarda mas que esto en llegar ya la ha abortado la pagina
RETRASO_MAX_S = 0.3
TOLERANCIA_PRESET_DEG = 2.0
PRESET_MAX_S = 40.0
MAX_CUERPO = 1024           # bytes

RUTAS_POST = ('/mando', '/emergencia', '/rearmar')
CABECERA_CLAVE = 'X-StarCrawler-Token'
CLAVES = frozenset(('sesion', 'seq', 'avance', 'giro', 'lento',
                    'delanteras', 'traseras', 'inclinar', 'preset'))

# Estados del preset: los dos primeros siguen vivos, el resto es el motivo
ESPERANDO = 'esperando'
EN_CURSO = 'en curso'
LLEGADO = 'llegado'
CANCELADO = 'cancelado'
SIN_ENCODER = 'sin encoder'
SIN_ENLACE = 'sin enlace'
TIEMPO = 'tiempo'
_VIVOS = (ESPERANDO, EN_CURSO)

_SESION = re.compile(r'[A-Za-z0-9]{8,32}')
_ENTERO = re.compile(r'[0-9]{1,18}')


# ─── Cuerpo de POST /mando ───────────────────────────────────────────────────

@dataclass(frozen=True)
class OrdenWeb:
    sesion: str
    seq: int
    avance: float
    giro: float                 # + = a la derecha
    lento: bool
    delanteras: int             # -1 | 0 | 1
    traseras: int
    inclinar: Optional[str]     # None | arriba | abajo | izq | der
    preset: Optional[int]       # None | 0..3

    def manual(self) -> bool:
        """Mueve orugas a mano: pares o cruceta."""
        return bool(self.delanteras or self.traseras or self.inclinar)

    def neutra(self) -> bool:
        return (self.avance == 0 and self.giro == 0 and not self.manual()
                and self.preset is None)


def _rechazar_constante(nombre: str):
    raise ValueError(nombre)


def _sin_repetidas(pares):
    d = dict(pares)
    if len(d) != len(pares):
        raise ValueError('clave repetida')
    return d


def _num(valor, nombre: str) -> float:
    # int antes que isfinite: un entero enorme no cabe en un float
    if type(valor) is int:
        ok = -1 <= valor <= 1
    elif type(valor) is float:
        ok = math.isfinite(valor) and -1.0 <= valor <= 1.0
    else:
        ok = False
    if not ok:
        raise ValueError('%s: numero en [-1, 1]' % nombre)
    return float(valor)


def _sentido(valor, nombre: str) -> int:
    if type(valor) is not int or valor not in (-1, 0, 1):
        raise ValueError('%s: -1, 0 o 1' % nombre)
    return valor


def validar_mando(cuerpo: bytes) -> OrdenWeb:
    """Cuerpo de POST /mando -> OrdenWeb. ValueError ante cualquier fallo."""
    if len(cuerpo) > MAX_CUERPO:
        raise ValueError('cuerpo demasiado largo')
    try:
        datos = json.loads(cuerpo, parse_constant=_rechazar_constante,
                           object_pairs_hook=_sin_repetidas)
    except (ValueError, TypeError, RecursionError):
        raise ValueError('JSON no valido') from None
    if type(datos) is not dict or set(datos) != CLAVES:
        raise ValueError('claves: ' + ', '.join(sorted(CLAVES)))

    sesion = datos['sesion']
    if type(sesion) is not str or not _SESION.fullmatch(sesion):
        raise ValueError('sesion: 8 a 32 letras o cifras')
    seq = datos['seq']
    if type(seq) is not int or seq < 0:
        raise ValueError('seq: entero >= 0')
    lento = datos['lento']
    if type(lento) is not bool:
        raise ValueError('lento: true o false')
    inclinar = datos['inclinar']
    if inclinar is not None and (type(inclinar) is not str
                                 or inclinar not in INCLINACIONES):
        raise ValueError('inclinar: null o ' + ', '.join(INCLINACIONES))
    preset = datos['preset']
    if preset is not None and (type(preset) is not int
                               or not 0 <= preset < len(PRESETS_DEG)):
        raise ValueError('preset: null o 0..%d' % (len(PRESETS_DEG) - 1))

    return OrdenWeb(
        sesion=sesion, seq=seq,
        avance=_num(datos['avance'], 'avance'),
        giro=_num(datos['giro'], 'giro'),
        lento=lento,
        delanteras=_sentido(datos['delanteras'], 'delanteras'),
        traseras=_sentido(datos['traseras'], 'traseras'),
        inclinar=inclinar, preset=preset)


# ─── Cabeceras, antes de leer el cuerpo ──────────────────────────────────────

def _cabecera(cabeceras, nombre: str) -> Optional[str]:
    """Sin distinguir mayusculas. ValueError si viene repetida."""
    nombre = nombre.lower()
    valores = [v for k, v in cabeceras.items() if k.lower() == nombre]
    if len(valores) > 1:
        raise ValueError('%s repetida' % nombre)
    return valores[0] if valores else None


def longitud_cuerpo(cabeceras) -> Optional[int]:
    """Content-Length, o None si falta. ValueError si no es un entero."""
    valor = _cabecera(cabeceras, 'Content-Length')
    if valor is None:
        return None
    valor = valor.strip()
    if not _ENTERO.fullmatch(valor):
        raise ValueError('Content-Length no valida')
    return int(valor)


def clave_valida(dada: Optional[str], clave: str) -> bool:
    if not clave or dada is None:
        return False
    return hmac.compare_digest(dada.encode('utf-8', 'replace'),
                               clave.encode('utf-8', 'replace'))


def comprobar_cabeceras(ruta: str, cabeceras, mando: bool,
                        clave: str) -> Optional[Tuple[int, str]]:
    """None si se puede leer el cuerpo; si no, (codigo, motivo)."""
    if ruta not in RUTAS_POST:
        return 404, 'ruta desconocida'
    if not mando:
        return 403, 'mando web desactivado (gui_mando:=true)'
    if ruta != '/emergencia':
        try:
            dada = _cabecera(cabeceras, CABECERA_CLAVE)
        except ValueError:
            dada = None
        if not clave_valida(dada, clave):
            return 401, 'clave'
    if ruta == '/mando':
        try:
            tipo = _cabecera(cabeceras, 'Content-Type') or ''
        except ValueError:
            tipo = ''
        if tipo.split(';')[0].strip().lower() != 'application/json':
            return 415, 'Content-Type: application/json'
    try:
        n = longitud_cuerpo(cabeceras)
    except ValueError as e:
        return 400, str(e)
    if n is None:
        # parar no se niega por una formalidad
        if ruta == '/emergencia':
            return None
        return 411, 'falta Content-Length'
    if n > MAX_CUERPO:
        return 413, 'cuerpo de mas de %d bytes' % MAX_CUERPO
    if ruta == '/mando' and n <= 0:
        return 411, 'cuerpo vacio'
    return None


# ─── Arbitro ─────────────────────────────────────────────────────────────────

@dataclass
class Consigna:
    lineal: float = 0.0
    angular: float = 0.0
    incremento: List[int] = field(default_factory=lambda: [0] * N_ORUGAS)
    usar_posicion: bool = False
    objetivo_rad: List[float] = field(
        default_factory=lambda: [0.0] * N_ORUGAS)
    emergencia: bool = False


@dataclass
class _Preset:
    k: int
    t0: float
    estado: str = ESPERANDO
    t_curso: float = 0.0


class Arbitro:
    """Decide que orden web vale y que consigna sale de ella.

    Sin hilos ni reloj propio: se le pasa t (time.monotonic) y quien lo usa
    lo protege con un Lock. Lo que hay que llevar al log se acumula y se
    recoge con sacar_eventos().
    """

    def __init__(self, max_lineal: float = MAX_LINEAL,
                 max_angular: float = MAX_ANGULAR,
                 factor_lento: float = FACTOR_LENTO,
                 s_preset: float = S_PRESET,
                 preset_max_s: float = PRESET_MAX_S) -> None:
        self.max_lineal = max_lineal
        self.max_angular = max_angular
        self.factor_lento = factor_lento
        self.s_preset = s_preset
        self.preset_max_s = preset_max_s
        self._orden: Optional[OrdenWeb] = None
        self._t_orden = 0.0
        self._dueno: Optional[str] = None
        self._ip = ''
        self._t_dueno = 0.0
        self._seq = -1
        self._t_emergencia: Optional[float] = None
        self._preset: Optional[_Preset] = None
        self._ultimo_preset: Optional[int] = None
        self._pedido = (0.0, 0.0)
        self._eventos: List[str] = []

    # ─── Entradas ────────────────────────────────────────────────────────

    def mando(self, orden: OrdenWeb, ip: str, t: float, enlace: bool,
              activa: str, mux_ok: bool,
              t_conexion: Optional[float] = None) -> Tuple[int, str]:
        """t_conexion: cuando se acepto la conexion de este POST."""
        if activa == 'emergencia' or self._en_emergencia(t):
            return 409, 'emergencia'
        if t_conexion is not None:
            # Retenida en la red: no puede revivir tras una emergencia
            if (self._t_emergencia is not None
                    and t_conexion <= self._t_emergencia):
                return 409, 'orden anterior a la emergencia'
            if t - t_conexion > RETRASO_MAX_S:
                return 409, 'orden atrasada'
        if not mux_ok:
            return 503, 'sin mux'
        if orden.sesion != self._dueno:
            if self._dueno is not None and t - self._t_dueno < SUELTA_DUENO_S:
                return 409, 'otro puesto'
            self._eventos.append('Mando web: manda %s desde %s'
                                 % (orden.sesion[:6], ip))
            self._dueno, self._seq = orden.sesion, -1
            self._orden = None
            self._olvidar_preset()
        if orden.seq <= self._seq:
            return 409, 'seq no crece'
        self._seq, self._t_dueno, self._ip = orden.seq, t, ip
        if not enlace and not orden.neutra():
            self._orden = None
            return 503, 'sin enlace'
        if not self._fresca(t):
            self._olvidar_preset()
        self._seguir_preset(orden, t)
        self._orden, self._t_orden = orden, t
        return 200, 'ok'

    def emergencia(self, t: float, ip: str = '') -> None:
        if not self._en_emergencia(t):
            self._eventos.append('Emergencia web desde %s' % (ip or '?'))
        self._orden = None
        self._dueno = None
        self._olvidar_preset()
        self._t_emergencia = t

    # ─── Salidas ─────────────────────────────────────────────────────────

    def salida(self, t: float, robot) -> Optional[Consigna]:
        """robot: {elev[4] rad, encoder_ok[4], enlace}. None = no publicar."""
        if self._en_emergencia(t):
            self._pedido = (0.0, 0.0)
            return Consigna(emergencia=True)
        if not self._fresca(t):
            self._orden = None
            self._olvidar_preset()
            self._pedido = (0.0, 0.0)
            return None
        o = self._orden
        factor = self.factor_lento if o.lento else 1.0
        lineal, angular = traccion(o.avance, o.giro, factor,
                                   self.max_lineal, self.max_angular)
        c = Consigna(lineal, angular, componer_incremento(
            o.delanteras, o.traseras, inclinacion(o.inclinar)))
        self._aplicar_preset(c, t, robot)
        self._pedido = (lineal, angular)
        return c

    def estado(self, t: float, activa: str = '',
               mux_ok: bool = False) -> dict:
        """ESTADO['mando'] para /events."""
        dueno = None
        if self._dueno is not None and t - self._t_dueno < SUELTA_DUENO_S:
            dueno = {'id': self._dueno[:6], 'ip': self._ip}
        preset = None
        if self._preset is not None:
            p = self._preset
            preset = {'k': p.k, 'objetivo_deg': PRESETS_DEG[p.k],
                      'estado': p.estado}
        return {
            'habilitado': True,
            's_preset': self.s_preset,
            'activa': activa,
            'dueno': dueno,
            'preset': preset,
            'pedido': list(self._pedido),
            'mux_ok': bool(mux_ok),
        }

    def sacar_eventos(self) -> List[str]:
        eventos, self._eventos = self._eventos, []
        return eventos

    # ─── Interno ─────────────────────────────────────────────────────────

    def _en_emergencia(self, t: float) -> bool:
        return (self._t_emergencia is not None
                and t - self._t_emergencia < VENTANA_EMERGENCIA_S)

    def _fresca(self, t: float) -> bool:
        return self._orden is not None and t - self._t_orden < FRESCURA_S

    def _olvidar_preset(self) -> None:
        self._preset = None
        self._ultimo_preset = None

    def _terminar_preset(self, motivo: str) -> None:
        p = self._preset
        if p is None or p.estado not in _VIVOS:
            return
        if p.estado == EN_CURSO:
            self._eventos.append('Preset %+.0f deg: %s'
                                 % (PRESETS_DEG[p.k], motivo))
        p.estado = motivo

    def _seguir_preset(self, orden: OrdenWeb, t: float) -> None:
        # El temporizador arranca con cada valor nuevo; uno terminado se
        # ignora hasta que llegue otro
        if orden.preset != self._ultimo_preset:
            self._ultimo_preset = orden.preset
            if orden.preset is None:
                self._terminar_preset(CANCELADO)
            else:
                self._preset = _Preset(orden.preset, t)
        if orden.manual():
            self._terminar_preset(CANCELADO)

    def _aplicar_preset(self, c: Consigna, t: float, robot) -> None:
        p = self._preset
        if p is None or p.estado not in _VIVOS:
            return
        if p.estado == ESPERANDO:
            # Mantenido de verdad: una orden con el preset tras s_preset, no
            # la gracia de frescura de la ultima
            if self._t_orden - p.t0 < self.s_preset:
                return
            p.estado, p.t_curso = EN_CURSO, t
        objetivo = PRESETS_DEG[p.k]
        if not robot['enlace']:
            fin = SIN_ENLACE
        elif not all(robot['encoder_ok']):
            fin = SIN_ENCODER
        elif all(abs(math.degrees(a) - objetivo) <= TOLERANCIA_PRESET_DEG
                 for a in robot['elev']):
            fin = LLEGADO
        elif t - p.t_curso > self.preset_max_s:
            fin = TIEMPO
        else:
            c.usar_posicion = True
            c.objetivo_rad = [math.radians(objetivo)] * N_ORUGAS
            c.incremento = [0] * N_ORUGAS
            return
        self._terminar_preset(fin)
