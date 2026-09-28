"""
chasis_node.py — el chasis sobre sus orugas, para RViz y la vista 3D
====================================================================
Con cada /joint_states de las orugas publica las tres juntas virtuales del
chasis (altura, cabeceo, balanceo), con el mismo sello, y robot_state_publisher
las convierte en TF. La geometria la lee de /robot_description: no hay cotas
copiadas aqui. Modelo en chasis_core.py.

    Suscribe    /robot_description  std_msgs/String (latcheado)
                /joint_states       sensor_msgs/JointState
    Publica     /joint_states       las tres juntas de chasis_core.JUNTAS_CHASIS

Es una estimacion en suelo llano, valga para el simulador o para el robot:
con una IMU, cabeceo y balanceo saldrian de ella.
"""
from __future__ import annotations

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .chasis_core import (JUNTAS_CHASIS, JUNTAS_ORUGA, geometria_desde_urdf,
                          pose_chasis)


class NodoChasis(Node):

    def __init__(self):
        super().__init__('starcrawler_chasis')
        self.geo = None
        self.elevacion = {}
        self.ultima = [0.0, 0.0, 0.0]
        self.t_ultima = None

        self.create_subscription(
            String, 'robot_description', self.cb_descripcion,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            JointState, 'joint_states', self.cb_juntas, qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Sin estado de las orugas, la TF de base_link no existiria
        self.create_timer(0.2, self.cb_reposo)

    def cb_descripcion(self, msg: String):
        try:
            self.geo = geometria_desde_urdf(msg.data)
        except (ValueError, AttributeError, KeyError) as e:
            self.get_logger().error('URDF sin la geometria de las orugas: %s' % e)
            return
        self.get_logger().info(
            'Chasis: poleas %.4f/%.4f m a %.3f m, reposo a %.4f m'
            % (self.geo.radio_polea, self.geo.radio_punta, self.geo.largo,
               self.geo.altura_reposo))

    def cb_juntas(self, msg: JointState):
        nuevas = False
        for nombre, q in zip(msg.name, msg.position):
            if nombre in JUNTAS_ORUGA:
                self.elevacion[nombre] = q
                nuevas = True
        if not nuevas or self.geo is None or len(self.elevacion) < 4:
            return
        pose = pose_chasis([self.elevacion[n] for n in JUNTAS_ORUGA], self.geo)
        self.ultima = [pose.altura - self.geo.altura_reposo,
                       pose.cabeceo, pose.balanceo]
        self.publicar(msg.header.stamp)

    def cb_reposo(self):
        ahora = self.get_clock().now()
        if self.t_ultima is None or (ahora - self.t_ultima).nanoseconds > 5e8:
            self.publicar(ahora.to_msg())

    def publicar(self, sello):
        out = JointState()
        out.header.stamp = sello
        out.name = list(JUNTAS_CHASIS)
        out.position = list(self.ultima)
        self.pub.publish(out)
        self.t_ultima = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoChasis()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # el launch reenvia SIGINT durante el cierre: no interrumpirlo
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
