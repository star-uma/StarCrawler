"""fisica_node: mundo_node con la fisica de MuJoCo en vez del contacto
cuasiestatico.

Mismas entradas (/starcrawler/state, /robot_description, /initialpose) y
mismas salidas (TF odom -> base_footprint, juntas del chasis, /mundo/verdad,
/mundo/estado, marcadores e indicadores). Lo que cambia es la planta: el
robot es un solido libre con sus brazos y orugas, y vuelca, patina, se
queda colgado o rebota de verdad. Con visor:=true abre el visor de MuJoCo.
"""

from __future__ import annotations

import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from starcrawler_odometry.odometry_core import normalizar_angulo

from . import fisica_core
from .mundo_node import AVISO_MS, DT_MAX_S, NodoMundo


class NodoFisica(NodoMundo):

    def __init__(self):
        self.fisica = None
        self.reconstruir = True
        self.visor = None
        super().__init__()
        self.declare_parameter('paso_s', fisica_core.ParFisica.paso)
        self.declare_parameter('rozamiento', fisica_core.ParFisica.rozamiento)
        self.declare_parameter('visor', False)
        self.par = fisica_core.ParFisica(
            paso=float(self.get_parameter('paso_s').value),
            rozamiento=float(self.get_parameter('rozamiento').value))
        self.con_visor = bool(self.get_parameter('visor').value)

    # El mundo o el URDF cambian: se rehace el modelo en el siguiente tick
    def cambiar_mundo(self, mundo):
        self.reconstruir = True
        super().cambiar_mundo(mundo)

    def cb_descripcion(self, msg):
        self.reconstruir = True
        super().cb_descripcion(msg)

    def colocar(self, x, y, yaw):
        self.pendiente = (x, y, yaw)
        if self.fisica is not None and not self.reconstruir:
            self.fisica.colocar(x, y, yaw, self.elevaciones)
            self.estado = self.fisica.estado()

    def preparar(self):
        x, y, yaw = self.aqui()
        t0 = time.perf_counter()
        self.fisica = fisica_core.Fisica(self.geo, self.mundo, self.par)
        self.fisica.colocar(x, y, yaw, self.elevaciones)
        self.estado = self.fisica.estado()
        self.reconstruir = False
        m = self.fisica.modelo
        self.get_logger().info(
            'Fisica lista en %.0f ms: %d cuerpos, %d geoms, paso %.1f ms'
            % ((time.perf_counter() - t0) * 1e3, m.nbody, m.ngeom, m.opt.timestep * 1e3))
        if self.con_visor:
            self.abrir_visor()

    def abrir_visor(self):
        if self.visor is not None:
            self.visor.close()
        try:
            import mujoco.viewer
            self.visor = mujoco.viewer.launch_passive(self.fisica.modelo,
                                                      self.fisica.datos)
        except Exception as e:  # sin pantalla: sigue sin visor
            self.get_logger().error('No puedo abrir el visor de MuJoCo: %s' % e)
            self.con_visor = False
            self.visor = None

    def cb_tick(self):
        ahora = self.get_clock().now()
        dt = min(max((ahora - self.t_tick).nanoseconds / 1e9, 0.0), DT_MAX_S)
        self.t_tick = ahora
        sello = ahora.to_msg()
        if self.geo is None:
            self.publicar_tf(sello, 0.0, 0.0, 0.0)
            self.publicar_juntas(sello, [0.0, 0.0, 0.0])
            return
        if self.reconstruir or self.fisica is None:
            self.preparar()

        fresco = (self.t_estado is not None
                  and (ahora - self.t_estado).nanoseconds <= self.timeout_ns)
        v, w = (self.v, self.w) if fresco else (0.0, 0.0)
        mitad = 0.5 * w * self.separacion
        antes = self.estado.pose
        t0 = time.perf_counter()
        self.estado = self.fisica.paso(dt, v - mitad, v + mitad, self.elevaciones)
        self.ms = (time.perf_counter() - t0) * 1e3
        if self.ms > AVISO_MS + 1e3 * dt:
            self.get_logger().warn('La fisica va mas lenta que el reloj: %.1f ms por '
                                   'tick de %.0f ms' % (self.ms, 1e3 * dt),
                                   throttle_duration_sec=5.0)
        if self.visor is not None and self.visor.is_running():
            self.visor.sync()

        e = self.estado
        p = e.pose
        self.patinado += max(0.0, abs(e.propuesto) - abs(e.avance))
        giro = normalizar_angulo(p.yaw - antes.yaw)
        self.ventana[0] += e.avance
        self.ventana[1] += dt
        self.publicar_tf(sello, p.x, p.y, p.yaw)
        self.publicar_juntas(sello, [p.z - self.geo.orugas.altura_reposo,
                                     p.cabeceo, p.balanceo])
        self.publicar_verdad(sello, p, e.avance / dt if dt > 0.0 else 0.0,
                             giro / dt if dt > 0.0 else 0.0)
        self.ticks += 1
        if self.ticks % self.cada == 0:
            self.publicar_estado(sello, v)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoFisica()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if nodo.visor is not None:
            nodo.visor.close()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
