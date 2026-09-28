#!/usr/bin/env python3
"""
crawler_mux_node.py — multiplexor de /crawler/command
=====================================================
El twist_mux de las orugas. Lee la tabla de fuentes de config/mux.yaml
(seccion crawler_mux, parametros topics.<fuente>.{topic, timeout, priority,
engancha}) y:

    Suscribe    <topic> de cada fuente     starcrawler_msgs/CrawlerCommand
                crawler_mux/rearmar        std_msgs/Empty

    Publica     crawler/command            starcrawler_msgs/CrawlerCommand
                crawler_mux/activa         std_msgs/String (transient_local,
                                           solo al cambiar)

La logica esta en mux_core.py (modulo puro con tests). Si el nodo se
reinicia pierde el enganche de la emergencia (ver docs/ros2.md).
"""
from __future__ import annotations

import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

from starcrawler_msgs.msg import CrawlerCommand

from .mux_core import Mux, Orden, leer_fuentes

PERIODO_TICK_S = 0.05     # 20 Hz


class NodoCrawlerMux(Node):

    def __init__(self) -> None:
        # Como twist_mux: los topics.* anidados se declaran solos desde el yaml
        super().__init__(
            'crawler_mux',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)

        parametros = {n: p.value for n, p in
                      self.get_parameters_by_prefix('topics').items()}
        fuentes = leer_fuentes(parametros)
        self.mux = Mux(f for f, _ in fuentes)
        self._activa: str | None = None

        self.pub = self.create_publisher(CrawlerCommand, 'crawler/command', 10)
        self.pub_activa = self.create_publisher(
            String, 'crawler_mux/activa',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        for fuente, topico in fuentes:
            self.create_subscription(
                CrawlerCommand, topico,
                lambda msg, n=fuente.nombre: self.cb_fuente(n, msg), 10)
        self.create_subscription(
            Empty, 'crawler_mux/rearmar', self.cb_rearmar, 10)
        self.create_timer(PERIODO_TICK_S, self.cb_tick)

        self.get_logger().info('crawler_mux: ' + ', '.join(
            f'{f.nombre} <- {topico} ({f.prioridad}, {f.timeout_s:g} s'
            + (', engancha)' if f.engancha else ')')
            for f, topico in fuentes))

    def ahora(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_fuente(self, nombre: str, msg: CrawlerCommand) -> None:
        orden = Orden(
            incremento=tuple(int(v) for v in msg.increment),
            usar_posicion=bool(msg.use_position),
            objetivo_rad=tuple(float(v) for v in msg.target),
            emergencia=bool(msg.emergency_stop))
        t = self.ahora()
        enganchado = self.mux.enganchado
        salida = self.mux.recibir(nombre, orden, t)
        if salida is not None:
            self.publicar(salida)
        if self.mux.enganchado and not enganchado:
            self.get_logger().warn(f'Emergencia enganchada por {nombre}.')
            self.publicar_activa(t)

    def cb_tick(self) -> None:
        t = self.ahora()
        salida = self.mux.tick(t)
        if salida is not None:
            self.publicar(salida)
        self.publicar_activa(t)

    def cb_rearmar(self, _msg: Empty) -> None:
        t = self.ahora()
        enganchado = self.mux.enganchado
        if not self.mux.rearmar(t):
            self.get_logger().warn(
                'Rearme rechazado: piden emergencia '
                + ', '.join(self.mux.piden_emergencia(t)) + '.')
        elif enganchado:
            self.get_logger().info('Rearmado: manda la fuente fresca.')
        else:
            self.get_logger().info('Rearme pedido sin emergencia enganchada.')
        self.publicar_activa(t)

    def publicar(self, orden: Orden) -> None:
        msg = CrawlerCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.increment = list(orden.incremento)
        msg.use_position = orden.usar_posicion
        msg.target = list(orden.objetivo_rad)
        msg.emergency_stop = orden.emergencia
        self.pub.publish(msg)

    def publicar_activa(self, t: float) -> None:
        activa = self.mux.activa(t)
        if activa != self._activa:
            self._activa = activa
            self.pub_activa.publish(String(data=activa))
            self.get_logger().debug(f'Manda: {activa or "nadie"}')


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        nodo = NodoCrawlerMux()
    except ValueError as e:
        get_logger('crawler_mux').fatal(f'mux.yaml: {e}')
        rclpy.shutdown()
        sys.exit(1)
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
