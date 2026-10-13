"""gazebo_node: mundo_node con Gazebo Fortress como planta.

Mismas entradas (/starcrawler/state, /robot_description, /initialpose) y
mismas salidas (TF odom -> base_footprint, juntas del chasis, /mundo/verdad,
/mundo/estado, marcadores) que mundo_node y fisica_node, asi que el mando, la
web, RViz y el nivelado no cambian. Manda a Gazebo la velocidad de las
ruedas de cada lado y la posicion de los brazos que da el firmware simulado,
y lee la pose verdadera de OdometryPublisher. /imu/data la da el sensor de
Gazebo por el puente, no este nodo.
"""

from __future__ import annotations

import math
import signal
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from starcrawler_odometry.odometry_core import normalizar_angulo
from starcrawler_sim.contacto_core import Estado, Pose3D
from starcrawler_sim.mundo_node import DT_MAX_S, NodoMundo

from . import sdf_core

VUELCO = math.radians(75.0)
V_CAIDA = 0.4               # m/s hacia abajo
T_BLOQUEO = 1.0             # s sin avanzar lo que mandan las orugas
FRACCION_BLOQUEO = 0.25


def _ypr(q):
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    cabeceo = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    balanceo = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    return yaw, cabeceo, balanceo


class NodoGazebo(NodoMundo):

    def __init__(self):
        self.odom = None
        self.listo = False
        super().__init__()
        self.pub_ruedas = {(lado, rueda): self.create_publisher(
            Float64, sdf_core.topico_rueda(lado, rueda), 10)
            for lado in sdf_core.LADOS for rueda in sdf_core.RUEDAS}
        self.pub_brazos = [self.create_publisher(Float64, sdf_core.topico_brazo(i), 10)
                           for i in range(4)]
        self.create_subscription(Odometry, sdf_core.TOPICO_ODOM, self.cb_odom, 10)
        self.anterior = None
        self.t_lento = 0.0
        self.listo = True

    # --- Gazebo ----------------------------------------------------------

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def publicar_imu(self, sello, p):
        pass                        # la da el sensor de Gazebo por el puente

    def colocar(self, x, y, yaw):
        self.pendiente = (x, y, yaw)
        if not self.listo or self.geo is None:
            return                  # al arrancar ya esta donde lo pone el SDF
        z = self.mundo.altura(x, y) + self.geo.orugas.altura_reposo + 0.05
        peticion = ('name: "%s", position: {x: %g, y: %g, z: %g}, '
                    'orientation: {z: %g, w: %g}'
                    % (sdf_core.MODELO, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
        subprocess.Popen(
            ['ign', 'service', '-s', '/world/%s/set_pose' % sdf_core.MODELO,
             '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '2000', '--req', peticion],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def radios(self):
        o = self.geo.orugas
        return {'polea': o.radio_polea, 'punta': o.radio_punta,
                'rodillo': min(0.03, 0.9 * o.radio_punta)}

    # --- El tick ---------------------------------------------------------

    def cb_tick(self):
        ahora = self.get_clock().now()
        dt = min(max((ahora - self.t_tick).nanoseconds / 1e9, 0.0), DT_MAX_S)
        self.t_tick = ahora
        sello = ahora.to_msg()
        if self.geo is None or self.odom is None:
            self.publicar_tf(sello, 0.0, 0.0, 0.0)
            self.publicar_juntas(sello, [0.0, 0.0, 0.0])
            return

        fresco = (self.t_estado is not None
                  and (ahora - self.t_estado).nanoseconds <= self.timeout_ns)
        v, w = (self.v, self.w) if fresco else (0.0, 0.0)
        mitad = 0.5 * w * self.separacion
        velocidad = {'izq': v - mitad, 'der': v + mitad}
        for (lado, rueda), pub in self.pub_ruedas.items():
            pub.publish(Float64(data=velocidad[lado] / self.radios()[rueda]))
        for i, pub in enumerate(self.pub_brazos):
            if math.isfinite(self.elevaciones[i]):
                pub.publish(Float64(data=float(self.elevaciones[i])))

        pp = self.odom.pose.pose
        yaw, cabeceo, balanceo = _ypr(pp.orientation)
        p = Pose3D(pp.position.x, pp.position.y, yaw, pp.position.z, cabeceo, balanceo)
        antes = self.anterior or p
        self.anterior = p
        avance = ((p.x - antes.x) * math.cos(p.yaw) + (p.y - antes.y) * math.sin(p.yaw))
        propuesto = v * dt
        lento = (abs(propuesto) > 1e-4 and avance * math.copysign(1.0, propuesto)
                 < FRACCION_BLOQUEO * abs(propuesto))
        self.t_lento = self.t_lento + dt if lento else 0.0
        volcado = abs(cabeceo) > VUELCO or abs(balanceo) > VUELCO
        vz = self.odom.twist.twist.linear.z
        self.estado = Estado(
            pose=p, elevaciones=tuple(self.elevaciones), contactos=(),
            apoya=(True,) * 4, holguras=(0.0,) * 4, panza=False,
            bloqueado=(('FR' if propuesto > 0 else 'RR')
                       if self.t_lento > T_BLOQUEO and not volcado else None),
            sin_traccion=False, cayendo=vz < -V_CAIDA and not volcado,
            volcado=volcado, atrapado=False,
            cdg=(p.x, p.y, p.z), poligono=(), margen=float('nan'),
            avance=avance, propuesto=propuesto)

        self.patinado += max(0.0, abs(propuesto) - abs(avance))
        giro = normalizar_angulo(p.yaw - antes.yaw)
        self.ventana[0] += avance
        self.ventana[1] += dt
        self.publicar_tf(sello, p.x, p.y, p.yaw)
        self.publicar_juntas(sello, [p.z - self.geo.orugas.altura_reposo, cabeceo, balanceo])
        self.publicar_verdad(sello, p, avance / dt if dt > 0.0 else 0.0,
                             giro / dt if dt > 0.0 else 0.0)
        self.ticks += 1
        if self.ticks % self.cada == 0:
            self.publicar_estado(sello, v)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoGazebo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
