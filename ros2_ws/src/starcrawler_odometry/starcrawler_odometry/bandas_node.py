"""bandas_node: mueve los tacos de las orugas en RViz y en la web.

    /robot_description  std_msgs/String   las juntas de tacos y su periodo
    /starcrawler/state  RobotState        velocidad de las bandas (rad/s en la polea)
    /joint_states       sensor_msgs/JointState  posicion de los tacos

Vale igual con el simulador, el ESP32 o el driver serie: solo mira el estado.
"""

from __future__ import annotations

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from starcrawler_msgs.msg import RobotState

from .bandas_core import Bandas, juntas_de_tacos


class NodoBandas(Node):

    def __init__(self):
        super().__init__('starcrawler_bandas')
        self.declare_parameter('wheel_radius', 0.0764)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('timeout_estado_s', 0.5)
        self.radio = float(self.get_parameter('wheel_radius').value)
        self.timeout_ns = int(self.get_parameter('timeout_estado_s').value * 1e9)
        self.bandas = None
        self.v = (0.0, 0.0)
        self.t_estado = None
        self.create_subscription(
            String, 'robot_description', self.cb_descripcion,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RobotState, 'starcrawler/state', self.cb_estado,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.t_tick = self.get_clock().now()
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.cb_tick)

    def cb_descripcion(self, msg: String):
        try:
            juntas = juntas_de_tacos(msg.data)
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.get_logger().error('URDF ilegible: %s' % e)
            return
        n = sum(len(v) for v in juntas.values())
        avance = self.bandas.avance if self.bandas else None
        self.bandas = Bandas(juntas)
        if avance:
            self.bandas.avance = avance
        self.get_logger().info('%d juntas de tacos en el URDF' % n)

    def cb_estado(self, msg: RobotState):
        self.v = (msg.track_speed_left * self.radio, msg.track_speed_right * self.radio)
        self.t_estado = self.get_clock().now()

    def cb_tick(self):
        ahora = self.get_clock().now()
        dt = min(max((ahora - self.t_tick).nanoseconds / 1e9, 0.0), 0.1)
        self.t_tick = ahora
        if self.bandas is None:
            return
        fresco = (self.t_estado is not None
                  and (ahora - self.t_estado).nanoseconds <= self.timeout_ns)
        v_izq, v_der = self.v if fresco else (0.0, 0.0)
        self.bandas.avanzar(v_izq, v_der, dt)
        nombres, valores = self.bandas.posiciones()
        if not nombres:
            return
        js = JointState()
        js.header.stamp = ahora.to_msg()
        js.name = nombres
        js.position = valores
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoBandas()
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
