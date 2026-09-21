"""
odometry_node.py — odometria de orugas a partir de /starcrawler/state
=====================================================================
Separada del driver y en su propio paquete, como en Donatello. Motivos:
se prueba y se sustituye sin tocar el driver, y deja sitio para fusionar
una IMU mas adelante sin reescribir nada.

    Suscripcion   /starcrawler/state  starcrawler_msgs/RobotState
    Publicacion   /odom               nav_msgs/Odometry
                  TF odom -> base_footprint  (opcional)

AVISO sobre la fiabilidad: esto es odometria de ruedas sobre ORUGAS. Al
girar, las bandas deslizan sobre el suelo por definicion, asi que el yaw
deriva rapido — mucho mas que en un robot de ruedas. Sirve para ver el
movimiento en RViz y como entrada a fusionar; no para saber donde esta el
robot despues de un rato. Por eso la covarianza del yaw va alta a
proposito y publish_tf se puede desactivar.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from starcrawler_msgs.msg import RobotState

from .odometry_core import (
    velocidades_del_robot,
    integrar,
    cuaternion_de_yaw,
)


class OdometriaOrugas(Node):

    def __init__(self):
        super().__init__('starcrawler_odometry')

        # Geometria. Los valores por defecto salen del CAD
        # (star-uma/SimulacionOrugas, Ejecutables/DefinicionParametros.m):
        # radio de polea activa 0.15279/2 y separacion lateral 0.524.
        # El radio EFECTIVO con la banda tensada no es el geometrico: hay
        # que refinarlo rodando y medir.
        self.declare_parameter('wheel_radius', 0.0764)
        self.declare_parameter('track_separation', 0.524)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 50.0)

        self.radio = self.get_parameter('wheel_radius').value
        self.separacion = self.get_parameter('track_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publicar_tf = self.get_parameter('publish_tf').value
        rate = self.get_parameter('rate_hz').value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0

        self.pub_odom = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            RobotState, 'starcrawler/state', self.cb_estado, 10)

        self.ultimo = self.get_clock().now()
        self.create_timer(1.0 / rate, self.actualizar)

        self.get_logger().info(
            'Odometria activa: radio %.4f m, separacion %.3f m'
            % (self.radio, self.separacion))
        if not self.publicar_tf:
            self.get_logger().info('publish_tf desactivado: solo /odom')

    def cb_estado(self, msg: RobotState):
        self.v, self.w = velocidades_del_robot(
            msg.track_speed_left, msg.track_speed_right,
            self.radio, self.separacion)

    def actualizar(self):
        ahora = self.get_clock().now()
        dt = (ahora - self.ultimo).nanoseconds / 1e9
        self.ultimo = ahora

        self.x, self.y, self.th = integrar(
            self.x, self.y, self.th, self.v, self.w, dt)

        qx, qy, qz, qw = cuaternion_de_yaw(self.th)
        cuaternion = Quaternion(x=qx, y=qy, z=qz, w=qw)

        if self.publicar_tf:
            t = TransformStamped()
            t.header.stamp = ahora.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = cuaternion
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = ahora.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = cuaternion
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        # Diagonal de covarianza. El yaw va deliberadamente alto: con
        # orugas deslizando es el termino en el que menos hay que confiar,
        # y asi cualquier filtro que fusione esto lo pondera bajo.
        odom.pose.covariance[0] = 0.05      # x
        odom.pose.covariance[7] = 0.05      # y
        odom.pose.covariance[35] = 0.5      # yaw
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.5

        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    nodo = OdometriaOrugas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
