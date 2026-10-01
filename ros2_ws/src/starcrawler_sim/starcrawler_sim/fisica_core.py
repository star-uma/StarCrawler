"""Fisica de StarCrawler con MuJoCo, sin ROS (lo usan fisica_node y los tests).

El chasis es un cuerpo libre y cada brazo cuelga de su pivote. La oruga es
una cadena de ruedas a lo largo de la banda (polea activa, polea de punta y
rodillos en los dos tramos) acopladas como una correa: una sola consigna de
velocidad y un solo tope de fuerza por oruga, el del RMD. Los brazos van por
posicion, rigidos como el sinfin. Devuelve el mismo contacto_core.Estado que
el modelo cuasiestatico, para que mundo_node, la web y RViz no cambien.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from .contacto_core import Contacto, Estado, GeometriaRobot, Pose3D

NOMBRES = ('FR', 'FL', 'RR', 'RL')
IZQUIERDA = (False, True, False, True)
ATAQUE_PARED = math.radians(60.0)


@dataclass(frozen=True)
class ParFisica:
    paso: float = 0.002                 # s
    rozamiento: float = 0.9             # goma con tacos sobre madera
    rozamiento_chasis: float = 0.4
    # Rodillos solapados: entre dos queda un pico de ~1,5 mm. Con huecos, un
    # canto se encaja entre dos rodillos que giran en sentidos opuestos ahi
    # y la oruga no trepa.
    separacion_rodillos: float = 0.02   # m entre centros
    radio_rodillo: float = 0.035        # tope; nunca mas que 0,9 R2
    fuerza_banda: float = 350.0         # N por oruga (par de pico del RMD-X8 / R1)
    kv_banda: float = 20000.0           # N s/m: el RMD da todo su par al atascarse
    par_brazo: float = 200.0            # N m (sinfin NMRV030 1:80 + 57HS112)
    kp_brazo: float = 5000.0            # N m/rad
    amortiguacion_brazo: float = 300.0
    masa_ruedas: float = 0.4            # kg de poleas y rodillos por oruga
    vuelco: float = math.radians(75.0)
    v_caida: float = 0.4                # m/s hacia abajo
    t_bloqueo: float = 1.0              # s sin avanzar lo que mandan las orugas
    fraccion_bloqueo: float = 0.25


def _n(v: Sequence[float]) -> str:
    return ' '.join('%.6g' % x for x in v)


def _ruedas(s: float, r1: float, r2: float, largo: float,
            par: ParFisica) -> List[Tuple[float, float, float]]:
    """(x, z, radio) de las ruedas de una oruga en el marco del brazo.

    s = +1 delantera (punta en +x), -1 trasera. La banda es tangente a las dos
    poleas; los rodillos tocan la banda desde dentro en ambos tramos.
    """
    ruedas = [(0.0, 0.0, r1), (s * largo, 0.0, r2)]
    rr = min(par.radio_rodillo, 0.9 * r2)
    seno = (r1 - r2) / largo
    coseno = math.sqrt(1.0 - seno * seno)
    for nz in (-coseno, coseno):
        nx = s * seno
        t1 = (r1 * nx, r1 * nz)
        t2 = (s * largo + r2 * nx, r2 * nz)
        tramo = math.hypot(t2[0] - t1[0], t2[1] - t1[1])
        n = int(tramo // par.separacion_rodillos)
        for k in range(1, n):
            f = k / n
            x = t1[0] + f * (t2[0] - t1[0]) - rr * nx
            z = t1[1] + f * (t2[1] - t1[1]) - rr * nz
            ruedas.append((x, z, rr))
    return ruedas


def _solidos_mjcf(mundo) -> Tuple[str, str]:
    """Mallas convexas de los solidos de mundo_core: planta rectangular,
    techo plano y paredes verticales hasta el suelo."""
    mallas, geoms = [], []
    for k, so in enumerate(getattr(mundo, 'solidos', ())):
        esquinas = so.esquinas()
        techos = [so.techo(x, y) for x, y in esquinas]
        if max(techos) < 0.002:
            continue
        vertices = []
        for (x, y), z in zip(esquinas, techos):
            vertices += [x, y, -0.01, x, y, max(z, 0.001)]
        mallas.append('<mesh name="s%d" vertex="%s"/>' % (k, _n(vertices)))
        geoms.append('<geom type="mesh" mesh="s%d" class="terreno"/>' % k)
    return '\n'.join(mallas), '\n'.join(geoms)


def construir_mjcf(geo: GeometriaRobot, mundo, par: ParFisica = ParFisica()) -> str:
    o = geo.orugas
    largo_c, ancho_c, alto_c = geo.chasis
    media = geo.ancho_oruga / 2.0
    brazos = []
    for i, (px, py) in enumerate(o.pivotes):
        s = 1.0 if px > 0 else -1.0
        ruedas = _ruedas(s, o.radio_polea, o.radio_punta, o.largo, par)
        m = par.masa_ruedas / len(ruedas)
        cuerpos = []
        for j, (x, z, r) in enumerate(ruedas):
            cuerpos.append(
                '<body name="r_%s_%d" pos="%s">'
                '<joint name="r_%s_%d" type="hinge" axis="0 1 0" armature="0.01"/>'
                '<geom type="cylinder" size="%.5g %.5g" quat="0.7071068 0.7071068 0 0" '
                'mass="%.5g" class="banda"/></body>'
                % (NOMBRES[i], j, _n((x, 0.0, z)), NOMBRES[i], j, r, media, m))
        brazos.append(
            '<body name="b_%s" pos="%s">'
            '<joint name="b_%s" type="hinge" axis="0 %g 0" range="-3.1 3.1" '
            'limited="false" damping="%g" armature="0.5"/>'
            '<geom type="box" size="%.5g %.5g %.5g" pos="%.5g 0 0" mass="%.5g" '
            'contype="0" conaffinity="0" group="3"/>%s</body>'
            % (NOMBRES[i], _n((px, py, 0.0)), NOMBRES[i], -s,
               par.amortiguacion_brazo, o.largo / 2, media, o.radio_punta,
               s * o.largo / 2, max(geo.masa_brazo - par.masa_ruedas, 0.1),
               ''.join(cuerpos)))

    actuadores = []
    for i in range(4):
        nombre = NOMBRES[i]
        ruedas = _ruedas(1.0, o.radio_polea, o.radio_punta, o.largo, par)
        # Cada rueda con su lazo de velocidad, todas a la velocidad de banda
        # (la consigna es v / r). Acopladas por igualdades, un motor muy
        # rigido en una sola rueda bombea energia a la cadena.
        for j, (_, _, r) in enumerate(ruedas):
            actuadores.append(
                '<velocity name="banda_%s_%d" joint="r_%s_%d" kv="%.6g" '
                'forcelimited="true" forcerange="%.6g %.6g"/>'
                % (nombre, j, nombre, j, par.kv_banda * r * r,
                   -par.fuerza_banda * r, par.fuerza_banda * r))
        actuadores.append(
            '<position name="brazo_%s" joint="b_%s" kp="%g" forcelimited="true" '
            'forcerange="%g %g"/>' % (nombre, nombre, par.kp_brazo,
                                      -par.par_brazo, par.par_brazo))

    mallas, terreno = _solidos_mjcf(mundo)
    return """<mujoco model="starcrawler">
<compiler angle="radian"/>
<option timestep="%g" integrator="implicitfast" cone="elliptic" impratio="5"/>
<default>
  <default class="terreno"><geom contype="1" conaffinity="2" friction="1 0.01 0.001"
    rgba="0.6 0.6 0.55 1"/></default>
  <default class="banda"><geom contype="2" conaffinity="1" condim="3"
    friction="%g 0.01 0.001" solref="0.005 1" rgba="0.1 0.1 0.1 1"/></default>
</default>
<asset>%s</asset>
<worldbody>
<geom name="suelo" type="plane" size="0 0 1" class="terreno"/>
%s
<body name="chasis" pos="0 0 %g">
<freejoint name="libre"/>
<geom name="chasis" type="box" size="%g %g %g" mass="%g" contype="2" conaffinity="1"
  friction="%g 0.01 0.001" solref="0.005 1" rgba="0.8 0.1 0.1 1"/>
%s
</body>
</worldbody>
<actuator>%s</actuator>
</mujoco>""" % (par.paso, par.rozamiento, mallas, terreno, o.altura_reposo,
                largo_c / 2, ancho_c / 2, alto_c / 2, geo.masa_chasis,
                par.rozamiento_chasis, '\n'.join(brazos), '\n'.join(actuadores))


def _ypr(R) -> Tuple[float, float, float]:
    yaw = math.atan2(R[1, 0], R[0, 0])
    cabeceo = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    balanceo = math.atan2(R[2, 1], R[2, 2])
    return yaw, cabeceo, balanceo


def _cuat_de_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)


def _envolvente(puntos):
    """Envolvente convexa en planta, antihoraria (cadena monotona)."""
    p = sorted(set(puntos))
    if len(p) < 3:
        return p

    def cruz(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    abajo, arriba = [], []
    for q in p:
        while len(abajo) >= 2 and cruz(abajo[-2], abajo[-1], q) <= 0:
            abajo.pop()
        abajo.append(q)
    for q in reversed(p):
        while len(arriba) >= 2 and cruz(arriba[-2], arriba[-1], q) <= 0:
            arriba.pop()
        arriba.append(q)
    return abajo[:-1] + arriba[:-1]


def margen(cdg_xy, poligono) -> float:
    """Distancia del CdG al borde del poligono de apoyo; + = dentro."""
    if len(poligono) < 3:
        return -1.0
    x, y = cdg_xy
    d = math.inf
    for (ax, ay), (bx, by) in zip(poligono, poligono[1:] + poligono[:1]):
        ex, ey = bx - ax, by - ay
        lado = math.hypot(ex, ey)
        if lado < 1e-9:
            continue
        d = min(d, (ex * (y - ay) - ey * (x - ax)) / lado)
    return d


class Fisica:
    """El robot en un mundo. paso() avanza dt con las consignas del firmware."""

    def __init__(self, geo: GeometriaRobot, mundo, par: ParFisica = ParFisica()):
        self.geo, self.mundo, self.par = geo, mundo, par
        self.modelo = mujoco.MjModel.from_xml_string(construir_mjcf(geo, mundo, par))
        self.datos = mujoco.MjData(self.modelo)
        m = self.modelo
        self.id_chasis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chasis')
        self.geom_chasis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'chasis')
        self.pieza_de_geom = {}
        for g in range(m.ngeom):
            cuerpo = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
            if cuerpo and cuerpo.startswith('r_'):
                self.pieza_de_geom[g] = NOMBRES.index(cuerpo.split('_')[1])
        self.pieza_de_geom[self.geom_chasis] = 'chasis'
        self.qadr_brazo = [m.jnt_qposadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, 'b_' + n)] for n in NOMBRES]
        ruedas = _ruedas(1.0, geo.orugas.radio_polea, geo.orugas.radio_punta,
                         geo.orugas.largo, par)
        self.act_banda = [[(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                              'banda_%s_%d' % (n, j)), r)
                           for j, (_, _, r) in enumerate(ruedas)] for n in NOMBRES]
        self.act_brazo = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'brazo_' + n)
                          for n in NOMBRES]
        self.t_lento = 0.0
        self.colocar(0.0, 0.0, 0.0, (0.0,) * 4)

    def colocar(self, x: float, y: float, yaw: float, elevaciones: Sequence[float]):
        """El robot en (x, y, yaw), un poco por encima del terreno, en reposo."""
        d = self.datos
        mujoco.mj_resetData(self.modelo, d)
        z = self.mundo.altura(x, y) + self._altura_libre(elevaciones) + 0.01
        d.qpos[0:3] = (x, y, z)
        d.qpos[3:7] = _cuat_de_yaw(yaw)
        for i, q in enumerate(elevaciones):
            d.qpos[self.qadr_brazo[i]] = q
            d.ctrl[self.act_brazo[i]] = q
        mujoco.mj_forward(self.modelo, d)
        self.t_lento = 0.0
        self.avance = self.propuesto = 0.0

    def _altura_libre(self, elevaciones) -> float:
        """Altura de los pivotes para que nada toque el suelo con estos brazos."""
        o = self.geo.orugas
        h = self.geo.chasis[2] / 2
        for q in elevaciones:
            # la punta baja L sin(-q) por debajo del pivote si el brazo baja
            h = max(h, o.radio_polea, o.largo * math.sin(-q) + o.radio_punta)
        return h

    def paso(self, dt: float, v_izq: float, v_der: float,
             elevaciones: Sequence[float]) -> Estado:
        """Avanza dt s con las bandas a v_izq/v_der m/s y los brazos hacia
        elevaciones (rad, positivo = levantado)."""
        m, d, par = self.modelo, self.datos, self.par
        for i in range(4):
            v = v_izq if IZQUIERDA[i] else v_der
            for a, r in self.act_banda[i]:
                d.ctrl[a] = v / r
            if math.isfinite(elevaciones[i]):
                d.ctrl[self.act_brazo[i]] = elevaciones[i]
        x0, y0 = d.qpos[0], d.qpos[1]
        n = max(1, int(round(dt / m.opt.timestep)))
        for _ in range(n):
            mujoco.mj_step(m, d)
        yaw = _ypr(d.xmat[self.id_chasis].reshape(3, 3))[0]
        self.avance = (d.qpos[0] - x0) * math.cos(yaw) + (d.qpos[1] - y0) * math.sin(yaw)
        self.propuesto = 0.5 * (v_izq + v_der) * n * m.opt.timestep
        lento = (abs(self.propuesto) > 1e-4 and
                 self.avance * math.copysign(1.0, self.propuesto)
                 < par.fraccion_bloqueo * abs(self.propuesto))
        self.t_lento = self.t_lento + n * m.opt.timestep if lento else 0.0
        return self.estado()

    def estado(self) -> Estado:
        m, d, par = self.modelo, self.datos, self.par
        R = d.xmat[self.id_chasis].reshape(3, 3)
        yaw, cabeceo, balanceo = _ypr(R)
        x, y, z = (float(v) for v in d.xpos[self.id_chasis])
        pose = Pose3D(x, y, yaw, z, cabeceo, balanceo)

        contactos = []
        for k in range(d.ncon):
            c = d.contact[k]
            pieza = self.pieza_de_geom.get(c.geom1, self.pieza_de_geom.get(c.geom2))
            if pieza is None or c.dist > 0.002:
                continue
            nrm = np.array(c.frame[0:3])
            if c.geom1 in self.pieza_de_geom:
                nrm = -nrm                      # terreno -> robot
            ataque = math.acos(max(-1.0, min(1.0, float(nrm[2]))))
            contactos.append(Contacto(
                NOMBRES[pieza] if pieza != 'chasis' else 'chasis',
                float(c.pos[0]), float(c.pos[1]), float(c.pos[2]),
                tuple(float(v) for v in nrm), ataque,
                'pared' if ataque > ATAQUE_PARED else 'apoyo',
                pieza != 'chasis', 0.0))
        apoya = tuple(any(c.pieza == n for c in contactos) for n in NOMBRES)
        panza = any(c.pieza == 'chasis' for c in contactos)

        holguras = []
        o = self.geo.orugas
        for i, n in enumerate(NOMBRES):
            if apoya[i]:
                holguras.append(0.0)
                continue
            px, py = o.pivotes[i]
            s = 1.0 if px > 0 else -1.0
            q = d.qpos[self.qadr_brazo[i]]
            punta = R @ np.array((px + s * o.largo * math.cos(q), py,
                                  o.largo * math.sin(q))) + d.xpos[self.id_chasis]
            holguras.append(max(0.0, float(punta[2]) - o.radio_punta
                                - self.mundo.altura(float(punta[0]), float(punta[1]))))

        cdg = tuple(float(v) for v in d.subtree_com[self.id_chasis])
        apoyos = [c for c in contactos if c.tipo == 'apoyo']
        envolvente = _envolvente([(round(c.x, 4), round(c.y, 4)) for c in apoyos])
        zs = {(round(c.x, 4), round(c.y, 4)): c.z for c in apoyos}
        poligono = tuple((px, py, zs[(px, py)]) for px, py in envolvente)
        marg = margen(cdg[:2], list(envolvente))

        volcado = abs(cabeceo) > par.vuelco or abs(balanceo) > par.vuelco or R[2, 2] < 0
        vz = float(d.qvel[2])
        cayendo = not contactos or vz < -par.v_caida
        bloqueado = None
        if self.t_lento > par.t_bloqueo and contactos:
            delante = ('FR', 'FL') if self.propuesto > 0 else ('RR', 'RL')
            paredes = [c for c in contactos if c.tipo == 'pared' and c.pieza in delante]
            if paredes:
                bloqueado = max(paredes, key=lambda c: c.z).pieza
            elif any(c.pieza == 'chasis' and c.tipo == 'pared' for c in contactos):
                bloqueado = 'chasis'
        sin_traccion = (self.t_lento > par.t_bloqueo and bloqueado is None
                        and panza and not any(apoya))
        return Estado(
            pose=pose,
            elevaciones=tuple(float(d.qpos[a]) for a in self.qadr_brazo),
            contactos=tuple(contactos), apoya=apoya, holguras=tuple(holguras),
            panza=panza, bloqueado=bloqueado, sin_traccion=sin_traccion,
            cayendo=cayendo and not volcado, volcado=volcado, atrapado=False,
            cdg=cdg, poligono=poligono, margen=marg,
            avance=float(self.avance), propuesto=float(self.propuesto))
