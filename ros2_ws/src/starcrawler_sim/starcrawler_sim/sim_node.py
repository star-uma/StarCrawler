"""
sim_node.py — el robot simulado, a nivel de tópicos
====================================================
Sustituye al robot entero: habla exactamente los mismos tópicos que
hablará el ESP32 con micro-ROS, así que el resto del grafo no distingue
si está el robot o esto.

    Suscribe    /cmd_vel           geometry_msgs/Twist
                /crawler/command   starcrawler_msgs/CrawlerCommand

    Publica     /starcrawler/state starcrawler_msgs/RobotState
                /joint_states      sensor_msgs/JointState

Con esto se levanta la cadena completa sin nada de hardware:

    mando -> twist_mux -> [sim] -> odometria -> GUI -> RViz

A diferencia de starcrawler_driver/simulator.py, que finge ser un puerto
serie, este no depende del protocolo: cuando el firmware pase a micro-ROS
aquel deja de servir y este sigue valiendo.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from starcrawler_msgs.msg import CrawlerCommand, RobotState

from .sim_core import (
    N_ORUGAS,
    RobotSimulado,
    elevacion_a_encoder_deg,
    encoder_a_elevacion_rad,
    es_espejada,
)

RAD_A_GRADOS = 57.29577951

NOMBRE_JOINT = [
    'crawler_fr_joint', 'crawler_fl_joint',
    'crawler_rr_joint', 'crawler_rl_joint',
]


class NodoSimulador(Node):

    def __init__(self):
        super().__init__('starcrawler_sim')

        # Geometria, igual que en el driver y la odometria
        self.declare_parameter('wheel_radius', 0.0764)
        self.declare_parameter('track_separation', 0.524)
        self.declare_parameter('rate_hz', 50.0)

        # Permite simular averias sin romper nada: util para comprobar que
        # el resto del grafo se comporta cuando el robot va degradado.
        self.declare_parameter('encoders_ok', True)
        self.declare_parameter('can_ok', True)

        self.radio = self.get_parameter('wheel_radius').value
        self.separacion = self.get_parameter('track_separation').value
        rate = self.get_parameter('rate_hz').value

        self.robot = RobotSimulado(
            encoders_ok=self.get_parameter('encoders_ok').value,
            can_ok=self.get_parameter('can_ok').value)

        self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(
            CrawlerCommand, 'crawler/command', self.cb_crawler, 10)

        self.pub_estado = self.create_publisher(RobotState, 'starcrawler/state', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)

        self.periodo = 1.0 / rate
        self.create_timer(self.periodo, self.actualizar)

        self.get_logger().info(
            'Robot simulado: nadie en el grafo notara que no hay hardware')

    # --- Entradas --------------------------------------------------------

    def cb_cmd_vel(self, msg: Twist):
        # Cinematica inversa del diferencial, igual que hace el firmware
        v_izq = msg.linear.x - (self.separacion / 2.0) * msg.angular.z
        v_der = msg.linear.x + (self.separacion / 2.0) * msg.angular.z
        self.robot.consigna_traccion(
            (v_izq / self.radio) * RAD_A_GRADOS,
            (v_der / self.radio) * RAD_A_GRADOS)

    def cb_crawler(self, msg: CrawlerCommand):
        # increment viene en convenio de elevacion (+1 sube el brazo); el
        # modelo trabaja en grados de encoder, donde subir puede ser
        # aumentar o disminuir segun el espejado de cada oruga.
        incrementos = []
        objetivos = []
        for i in range(N_ORUGAS):
            inc = int(msg.increment[i])
            incrementos.append(inc if es_espejada(i) else -inc)
            objetivos.append(elevacion_a_encoder_deg(i, float(msg.target[i])))

        self.robot.consigna_orugas(
            incrementos, objetivos, bool(msg.use_position),
            bool(msg.emergency_stop))

    # --- Salida ----------------------------------------------------------

    def actualizar(self):
        self.robot.avanzar(self.periodo)
        angulos, vel_izq, vel_der, seguridad, errores = self.robot.estado()

        ahora = self.get_clock().now().to_msg()
        elevaciones = [encoder_a_elevacion_rad(i, angulos[i])
                       for i in range(N_ORUGAS)]

        estado = RobotState()
        estado.header.stamp = ahora
        estado.crawler_angle = elevaciones
        estado.encoder_ok = [self.robot.encoders_ok] * N_ORUGAS
        estado.track_speed_left = vel_izq / RAD_A_GRADOS
        estado.track_speed_right = vel_der / RAD_A_GRADOS
        estado.can_ok = self.robot.can_ok
        estado.imu_ok = False
        estado.safety_active = seguridad
        estado.error_bits = errores
        estado.frames_ok = 0
        estado.frames_crc_error = 0
        self.pub_estado.publish(estado)

        joints = JointState()
        joints.header.stamp = ahora
        joints.name = NOMBRE_JOINT
        joints.position = elevaciones
        self.pub_joints.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoSimulador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
