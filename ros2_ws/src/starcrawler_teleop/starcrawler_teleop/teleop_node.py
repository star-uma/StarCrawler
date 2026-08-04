#!/usr/bin/env python3
"""
teleop_node.py — mando -> consignas ROS
=======================================
Lee /joy (nodo `joy`, mando conectado al PC de a bordo) y publica:

    /cmd_vel          geometry_msgs/Twist
    /crawler/command  starcrawler_msgs/CrawlerCommand

La logica esta en joy_logic.py (modulo puro con tests). Este nodo solo hace de
envoltorio ROS: carga el mapeo desde parametros y publica.

Publica a `rate_hz` fijo (no solo cuando llega /joy) para que el driver y el
watchdog del ESP32 vean un flujo constante de consignas.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from starcrawler_msgs.msg import CrawlerCommand

from .joy_logic import Ajustes, LogicaMando, Mapeo, N_ORUGAS


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
            ('dpad_botones', list(m.dpad_botones)),
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
        rate = float(self.get_parameter('rate_hz').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout_s').value)

        self.logica = LogicaMando(m, a)
        self.joy: Joy | None = None
        self.t_joy = 0.0
        self.aviso_dado = False

        self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_crawler = self.create_publisher(
            CrawlerCommand, 'crawler/command', 10)
        self.create_timer(1.0 / rate, self.publicar)

        self.get_logger().info(
            'Teleop listo (esquema simultaneo). SHARE = parada de emergencia.')

    def ahora(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_joy(self, msg: Joy) -> None:
        self.joy = msg
        self.t_joy = self.ahora()
        self.aviso_dado = False

    def publicar(self) -> None:
        t = self.ahora()
        vel = Twist()
        cmd = CrawlerCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self.joy is None or t - self.t_joy > self.joy_timeout:
            # Sin mando: ceros explicitos. El ESP32 para igual por watchdog,
            # esto solo evita repetir la ultima consigna.
            if self.joy is not None and not self.aviso_dado:
                self.get_logger().warn('Sin datos de /joy: enviando parada.')
                self.aviso_dado = True
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

        self.pub_vel.publish(vel)
        self.pub_crawler.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = StarCrawlerTeleop()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
