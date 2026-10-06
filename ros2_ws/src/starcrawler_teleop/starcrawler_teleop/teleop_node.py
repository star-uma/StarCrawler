#!/usr/bin/env python3
"""
teleop_node.py — mando -> consignas ROS
=======================================
Lee /joy (nodo `joy`, mando conectado al PC de a bordo) y publica:

    cmd_vel                  geometry_msgs/Twist
    crawler/command          starcrawler_msgs/CrawlerCommand
    cmd_vel_activo           lo mismo, solo mientras el DS4 esta tocado
    crawler/command_activo   (Salida.activo)

Con el nivelado activo (OPTIONS) lee tambien /imu/data y /starcrawler/state y
manda los brazos a la posicion que calcula nivelado_core.

Los nombres son relativos: el launch los remapea a las fuentes joy y
joy_activo de twist_mux y crawler_mux.

La logica esta en joy_logic.py (modulo puro con tests). Este nodo solo hace de
envoltorio ROS: carga el mapeo desde parametros y publica.

Publica a `rate_hz` fijo (no solo cuando llega /joy) para que el driver y el
watchdog del ESP32 vean un flujo constante de consignas.
"""
from __future__ import annotations

import math
import signal
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String
from starcrawler_msgs.msg import CrawlerCommand, RobotState

from .joy_logic import Ajustes, LogicaMando, Mapeo, N_ORUGAS
from .nivelado_core import AjustesNivelado, Nivelador, cabeceo_balanceo

# Mas viejos que esto, la IMU o el estado no valen para nivelar
FRESCURA_NIVELADO_S = 0.3


class StarCrawlerTeleop(Node):

    def __init__(self) -> None:
        super().__init__('starcrawler_teleop')

        # ─── Mapeo del mando (ver config/ds4.yaml) ───────────────────────
        m = Mapeo()
        for campo, valor in (
            ('eje_avance', m.eje_avance), ('eje_giro', m.eje_giro),
            ('invertir_avance', m.invertir_avance),
            ('invertir_giro', m.invertir_giro),
            ('boton_l1', m.boton_l1), ('boton_l2', m.boton_l2),
            ('boton_r1', m.boton_r1), ('boton_r2', m.boton_r2),
            ('eje_l2', m.eje_l2), ('eje_r2', m.eje_r2),
            ('boton_l3', m.boton_l3), ('boton_share', m.boton_share),
            ('boton_options', m.boton_options),
            ('botones_preset', list(m.botones_preset)),
            ('dpad_eje_x', m.dpad_eje_x), ('dpad_eje_y', m.dpad_eje_y),
            ('dpad_y_arriba_positivo', m.dpad_y_arriba_positivo),
            ('dpad_x_derecha_positivo', m.dpad_x_derecha_positivo),
            # rclpy no puede tipar una lista vacia: el sentinela [-1,-1,-1,-1]
            # significa "cruceta por ejes" (joy_logic ignora indices < 0)
            ('dpad_botones', [-1, -1, -1, -1]),
            ('boton_enable', m.boton_enable),
        ):
            self.declare_parameter(campo, valor)
            setattr(m, campo, self.get_parameter(campo).value)

        # ─── Ajustes ─────────────────────────────────────────────────────
        a = Ajustes()
        for campo, valor in (
            ('zona_muerta', a.zona_muerta), ('umbral_eje', a.umbral_eje),
            ('max_lineal', a.max_lineal), ('max_angular', a.max_angular),
            ('factor_lento', a.factor_lento), ('s_preset', a.s_preset),
        ):
            self.declare_parameter(campo, valor)
            setattr(a, campo, float(self.get_parameter(campo).value))

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('joy_timeout_s', 0.5)
        # Etiquetas de crawler_mux/activa que son este nodo
        self.declare_parameter('fuentes_propias', ['joy', 'joy_activo'])
        rate = float(self.get_parameter('rate_hz').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout_s').value)
        self.fuentes_propias = set(
            self.get_parameter('fuentes_propias').value)

        n = AjustesNivelado()
        for campo, valor in (('pivote_x', n.pivote_x), ('pivote_y', n.pivote_y),
                             ('ganancia_nivelado', n.ganancia)):
            self.declare_parameter(campo, valor)
        n.pivote_x = float(self.get_parameter('pivote_x').value)
        n.pivote_y = float(self.get_parameter('pivote_y').value)
        n.ganancia = float(self.get_parameter('ganancia_nivelado').value)
        self.nivelador = Nivelador(n)
        self.imu = None             # (cabeceo, balanceo, t)
        self.brazos = None          # (elevaciones rad, t)
        self.t_nivelado = None

        self.logica = LogicaMando(m, a)
        self.joy: Joy | None = None
        self.t_joy = 0.0
        self.aviso_dado = False

        self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        self.create_subscription(Imu, 'imu/data', self.cb_imu, qos_profile_sensor_data)
        self.create_subscription(RobotState, 'starcrawler/state', self.cb_estado,
                                 qos_profile_sensor_data)
        # crawler_mux la publica latcheada y solo al cambiar
        self.create_subscription(
            String, 'crawler_mux/activa', self.cb_activa,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_crawler = self.create_publisher(
            CrawlerCommand, 'crawler/command', 10)
        self.pub_vel_activo = self.create_publisher(
            Twist, 'cmd_vel_activo', 10)
        self.pub_crawler_activo = self.create_publisher(
            CrawlerCommand, 'crawler/command_activo', 10)
        self.create_timer(1.0 / rate, self.publicar)

        self.get_logger().info(
            'Teleop listo (esquema simultaneo). SHARE = parada de emergencia.')

    def ahora(self) -> float:
        # Reloj monotono: el del sistema en el WSL da saltos de segundos cada
        # ~35 s y haria creer que se ha perdido el mando (parada y sin pose)
        return time.monotonic()

    def cb_joy(self, msg: Joy) -> None:
        self.joy = msg
        self.t_joy = self.ahora()
        self.aviso_dado = False

    def cb_imu(self, msg: Imu) -> None:
        o = msg.orientation
        self.imu = (*cabeceo_balanceo(o.x, o.y, o.z, o.w), self.ahora())

    def cb_estado(self, msg: RobotState) -> None:
        # crawler_angle ya viene en radianes de elevacion; NaN si no hay encoder
        q = [float(msg.crawler_angle[i]) if msg.encoder_ok[i] else float('nan')
             for i in range(N_ORUGAS)]
        self.brazos = (q, self.ahora())

    def apagar_nivelado(self) -> None:
        if self.logica.nivelado:
            self.get_logger().info('Nivelado apagado: el mando ya no manda los brazos')
        self.logica.apagar_nivelado()
        self.nivelador.parar()
        self.t_nivelado = None

    def nivelar(self, t: float):
        """Objetivos de los brazos para nivelar, o None si falta la IMU o el
        estado (entonces los brazos se quedan donde esten)."""
        if (self.imu is None or self.brazos is None
                or t - self.imu[2] > FRESCURA_NIVELADO_S
                or t - self.brazos[1] > FRESCURA_NIVELADO_S):
            self.get_logger().warn('Nivelado sin IMU (/imu/data) o sin estado '
                                   'de los brazos: no nivelo',
                                   throttle_duration_sec=5.0)
            self.nivelador.parar()
            return None
        dt = 0.0 if self.t_nivelado is None else t - self.t_nivelado
        self.t_nivelado = t
        cabeceo, balanceo, _ = self.imu
        return self.nivelador.paso(cabeceo, balanceo, self.brazos[0], dt)

    def cb_activa(self, msg: String) -> None:
        # Si ha mandado otra fuente, la pose del DS4 no vuelve al soltarla
        if msg.data != '' and msg.data not in self.fuentes_propias:
            self.logica.cancelar_preset()
            self.apagar_nivelado()

    def publicar(self) -> None:
        t = self.ahora()

        # Sin haber visto NUNCA un mando: silencio absoluto. Asi no competimos
        # con ordenes manuales por topic (ros2 topic pub) ni con otros nodos.
        if self.joy is None:
            return

        vel = Twist()
        cmd = CrawlerCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if t - self.t_joy > self.joy_timeout:
            # Mando perdido a mitad de uso: ceros explicitos (el ESP32 para
            # igual por watchdog, esto solo evita repetir la ultima consigna).
            # Nada por el canal activo, y la pose enganchada se olvida.
            if not self.aviso_dado:
                self.get_logger().warn('Sin datos de /joy: enviando parada.')
                self.aviso_dado = True
            self.logica.cancelar_preset()
            self.apagar_nivelado()
            cmd.increment = [0] * N_ORUGAS
            self.pub_vel.publish(vel)
            self.pub_crawler.publish(cmd)
            return

        s = self.logica.procesar(self.joy.axes, self.joy.buttons, t)

        vel.linear.x = s.lineal
        vel.angular.z = s.angular

        cmd.increment = [int(v) for v in s.incremento]
        cmd.use_position = s.usar_posicion
        cmd.target = [float(v) for v in s.objetivo_rad]
        cmd.emergency_stop = s.emergencia

        objetivo = self.nivelar(t) if s.nivelar and not s.emergencia else None
        if objetivo is not None and all(math.isfinite(v) for v in objetivo):
            cmd.increment = [0] * N_ORUGAS
            cmd.use_position = True
            cmd.target = [float(v) for v in objetivo]
        elif not s.nivelar and self.nivelador.activo:
            self.nivelador.parar()
            self.t_nivelado = None

        self.pub_vel.publish(vel)
        self.pub_crawler.publish(cmd)
        if s.activo:
            self.pub_vel_activo.publish(vel)
            self.pub_crawler_activo.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = StarCrawlerTeleop()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # el launch reenvia SIGINT durante el cierre: no interrumpirlo
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
