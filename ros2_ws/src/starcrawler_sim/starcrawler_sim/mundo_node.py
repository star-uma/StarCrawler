"""
mundo_node.py — el robot simulado en un mundo con obstáculos
============================================================
Observa /starcrawler/state, venga de starcrawler_sim o del ESP32 en
HW_SIMULADO, apoya el robot en el terreno del YAML (mundo_core) con
contacto_core y publica la pose VERDADERA. Nada vuelve al robot: si choca, las
orugas patinan y /odom se le adelanta. robot.launch.py lo lanza con mundo:= en
lugar de chasis_node, y la odometría va entonces sin TF.

    Suscribe    /starcrawler/state   starcrawler_msgs/RobotState
                /robot_description   std_msgs/String (latcheado)
                /initialpose         geometry_msgs/PoseWithCovarianceStamped
    Publica     TF odom -> base_footprint (x, y, rumbo)
                /joint_states        chasis_core.JUNTAS_CHASIS: altura, cabeceo, balanceo
                /mundo/verdad        nav_msgs/Odometry, la pose 6D de base_link
                /mundo/marcadores    visualization_msgs/MarkerArray (latcheado)
                /mundo/indicadores   visualization_msgs/MarkerArray
                /mundo/estado        std_msgs/String con el JSON de marcadores.py
    Servicio    /mundo/reiniciar     std_srvs/Trigger: el robot a la salida

Sin el bit 7 de error_bits (robot real o driver serie) pasa a MODO ESPEJO:
borra el mundo y sigue dibujando el robot en llano hasta que se reinicie.
"""
from __future__ import annotations

import math
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import (PoseWithCovarianceStamped, Quaternion,
                               TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from starcrawler_msgs.msg import RobotState
from starcrawler_odometry.chasis_core import JUNTAS_CHASIS
from starcrawler_odometry.odometry_core import (cuaternion_de_yaw,
                                                normalizar_angulo,
                                                velocidades_del_robot)

from . import contacto_core, marcadores, mundo_core

BIT_SIMULADO = 7
DT_MAX_S = 0.1
AVISO_MS = 15.0
ESTADO_HZ = 10.0
MARCO_CHASIS = 'base_link'
MARCO_IMU = 'imu_link'


class MundoVacio:
    """Suelo llano sin nada: con un YAML que no carga y en modo espejo."""

    nombre = ''

    def altura(self, x, y):
        return 0.0

    def perfil(self, x0, y0, ux, uy, s0, s1):
        return [(s0, 0.0), (s1, 0.0)]

    def visual(self):
        return []


class NodoMundo(Node):

    def __init__(self):
        super().__init__('starcrawler_mundo')
        self.declare_parameter('mundo', '')
        self.declare_parameter('rate_hz', 50.0)
        # Los de la odometria: robot.launch.py los copia de odometry.yaml
        self.declare_parameter('wheel_radius', 0.0764)
        self.declare_parameter('track_separation', 0.524)
        self.declare_parameter('timeout_estado_s', 0.5)
        self.declare_parameter('recargar_auto', True)
        self.declare_parameter('marco_odom', 'odom')
        self.declare_parameter('marco_base', 'base_footprint')

        self.nombre = self.get_parameter('mundo').value
        rate = self.get_parameter('rate_hz').value
        self.radio = self.get_parameter('wheel_radius').value
        self.separacion = self.get_parameter('track_separation').value
        self.timeout_ns = int(self.get_parameter('timeout_estado_s').value * 1e9)
        self.recargar = self.get_parameter('recargar_auto').value
        self.marco_odom = self.get_parameter('marco_odom').value
        self.marco_base = self.get_parameter('marco_base').value

        self.mundo = MundoVacio()
        self.titulo = os.path.splitext(os.path.basename(self.nombre))[0]
        self.ruta = None
        self.version = 0
        self.valido = False
        self.firma = None
        self.ultimo_error = None
        self.aviso = None
        self.modo = 'normal'

        self.geo = None
        self.estado = None
        self.pendiente = (0.0, 0.0, 0.0)
        self.elevaciones = [0.0] * 4
        self.v = self.w = 0.0
        self.t_estado = None
        self.patinado = 0.0
        self.ms = 0.0
        self.ticks = 0
        self.cada = max(1, round(rate / ESTADO_HZ))
        self.ventana = [0.0, 0.0]       # avance real y tiempo desde el ultimo estado

        self.tf = TransformBroadcaster(self)
        self.pub_juntas = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_verdad = self.create_publisher(
            Odometry, 'mundo/verdad', QoSProfile(depth=10))
        self.pub_marcadores = self.create_publisher(
            MarkerArray, 'mundo/marcadores',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_indicadores = self.create_publisher(
            MarkerArray, 'mundo/indicadores', QoSProfile(depth=5))
        self.pub_estado = self.create_publisher(
            String, 'mundo/estado', QoSProfile(depth=10))
        # La IMU que tendria el robot: orientacion del chasis y gravedad
        self.pub_imu = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)

        # El ESP32 publica en best-effort: una suscripcion fiable no casa
        self.create_subscription(
            RobotState, 'starcrawler/state', self.cb_estado,
            qos_profile_sensor_data)
        self.create_subscription(
            String, 'robot_description', self.cb_descripcion,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.cb_initialpose,
            QoSProfile(depth=1))
        self.create_service(Trigger, 'mundo/reiniciar', self.cb_reiniciar)

        self.revisar_fichero()
        if self.recargar:
            self.create_timer(1.0, self.revisar_fichero)
        self.t_tick = self.get_clock().now()
        self.create_timer(1.0 / rate, self.cb_tick)

    # --- El mundo --------------------------------------------------------

    def revisar_fichero(self):
        """Carga el YAML si ha cambiado desde la ultima vez (por mtime)."""
        if self.modo == 'espejo':
            return
        try:
            if not self.nombre:
                raise ValueError("falta el parametro 'mundo'")
            ruta = self.ruta = mundo_core.resolver_ruta(self.nombre)
            firma = (ruta, os.stat(ruta).st_mtime_ns)
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.fallo(e)
            return
        if firma == self.firma:
            return
        self.firma = firma
        try:
            mundo = mundo_core.cargar_fichero(ruta)
        except Exception as e:  # ErrorMundo ya dice donde esta el fallo
            self.fallo(e)
            return
        self.valido = True
        self.aviso = None
        self.ultimo_error = None
        self.get_logger().info(
            'Mundo %s (%s)\n%s' % (mundo.nombre, ruta, mundo.resumen()))
        self.cambiar_mundo(mundo)

    def fallo(self, e):
        texto = (str(e) if isinstance(e, mundo_core.ErrorMundo)
                 else '%s: %s' % (type(e).__name__, e))
        if texto != self.ultimo_error:
            self.ultimo_error = texto
            self.get_logger().error(
                'Mundo %r no valido: %s%s' % (
                    self.nombre, texto,
                    '. Sigue el anterior' if self.valido else ''))
        if not self.valido:
            # En el chip de la web basta el nombre del fichero
            if self.ruta:
                texto = texto.replace(self.ruta, os.path.basename(self.ruta))
            self.aviso = ('mundo no válido: ' + texto,
                          'corrige el YAML: se carga al guardarlo'
                          if self.recargar else 'corrige el YAML y relanza')
        if self.version == 0:
            self.cambiar_mundo(MundoVacio())

    def cambiar_mundo(self, mundo):
        self.mundo = mundo
        self.titulo = mundo.nombre or self.titulo
        self.version += 1
        try:
            self.pub_marcadores.publish(marcadores.a_markers(
                mundo.visual(), self.marco_odom, self.get_clock().now().to_msg()))
        except Exception as e:  # un fallo de visual() no para el mundo
            self.get_logger().error('No puedo dibujar el mundo: %s' % e)
        self.colocar(*self.aqui())

    def entrar_en_espejo(self):
        self.modo = 'espejo'
        self.get_logger().error(
            'robot real o driver serie: el mundo se desactiva; el robot se '
            'dibuja en llano')
        self.cambiar_mundo(MundoVacio())

    def aqui(self):
        if self.estado is None:
            return self.pendiente
        p = self.estado.pose
        return p.x, p.y, p.yaw

    def colocar(self, x, y, yaw):
        self.pendiente = (x, y, yaw)
        if self.geo is not None:
            self.estado = contacto_core.colocar(
                x, y, yaw, tuple(self.elevaciones), self.mundo, self.geo)

    # --- Entradas --------------------------------------------------------

    def cb_estado(self, msg: RobotState):
        if self.modo != 'espejo' and not (msg.error_bits >> BIT_SIMULADO) & 1:
            self.entrar_en_espejo()
        # crawler_angle ya viene en radianes de elevacion
        for i in range(4):
            q = float(msg.crawler_angle[i])
            if bool(msg.encoder_ok[i]) and math.isfinite(q):
                self.elevaciones[i] = q
        v, w = velocidades_del_robot(msg.track_speed_left, msg.track_speed_right,
                                     self.radio, self.separacion)
        if math.isfinite(v) and math.isfinite(w):
            self.v, self.w = v, w
        self.t_estado = self.get_clock().now()

    def cb_descripcion(self, msg: String):
        try:
            geo = contacto_core.geometria_robot_desde_urdf(msg.data)
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.get_logger().error('URDF sin la geometria del robot: %s' % e)
            return
        self.geo = geo
        self.colocar(*self.aqui())
        self.get_logger().info('Geometria del robot leida de /robot_description')

    def cb_initialpose(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id.lstrip('/') != self.marco_odom:
            self.get_logger().warn(
                '/initialpose en %r y no en %r: ignorado'
                % (msg.header.frame_id, self.marco_odom))
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.colocar(p.x, p.y, yaw)
        self.get_logger().info(
            'Robot colocado en (%.2f, %.2f) a %.0f grados; /odom no se reinicia'
            % (p.x, p.y, math.degrees(yaw)))

    def cb_reiniciar(self, peticion, respuesta):
        self.colocar(0.0, 0.0, 0.0)
        respuesta.success = True
        respuesta.message = ('robot en la salida del mundo' if self.geo is not None
                             else 'se colocara en la salida al llegar el URDF')
        return respuesta

    # --- El tick ---------------------------------------------------------

    def cb_tick(self):
        ahora = self.get_clock().now()
        dt = min(max((ahora - self.t_tick).nanoseconds / 1e9, 0.0), DT_MAX_S)
        self.t_tick = ahora
        sello = ahora.to_msg()
        if self.estado is None:
            # Sin URDF todavia: como cb_reposo de chasis_node
            self.publicar_tf(sello, 0.0, 0.0, 0.0)
            self.publicar_juntas(sello, [0.0, 0.0, 0.0])
            return

        fresco = (self.t_estado is not None
                  and (ahora - self.t_estado).nanoseconds <= self.timeout_ns)
        v, w = (self.v, self.w) if fresco else (0.0, 0.0)
        antes = self.estado.pose
        t0 = time.perf_counter()
        self.estado = contacto_core.paso(
            self.estado, v, w, tuple(self.elevaciones), dt, self.mundo, self.geo)
        self.ms = (time.perf_counter() - t0) * 1e3
        if self.ms > AVISO_MS:
            self.get_logger().warn('El contacto tarda %.1f ms por tick' % self.ms,
                                   throttle_duration_sec=5.0)
        e = self.estado
        p = e.pose
        self.patinado += max(0.0, abs(e.propuesto) - abs(e.avance))
        avance = (p.x - antes.x) * math.cos(p.yaw) + (p.y - antes.y) * math.sin(p.yaw)
        giro = normalizar_angulo(p.yaw - antes.yaw)
        self.ventana[0] += avance
        self.ventana[1] += dt

        self.publicar_tf(sello, p.x, p.y, p.yaw)
        self.publicar_juntas(sello, [p.z - self.geo.orugas.altura_reposo,
                                     p.cabeceo, p.balanceo])
        self.publicar_verdad(sello, p, avance / dt if dt > 0.0 else 0.0,
                             giro / dt if dt > 0.0 else 0.0)
        self.publicar_imu(sello, p)
        self.ticks += 1
        if self.ticks % self.cada == 0:
            self.publicar_estado(sello, v)

    # --- Salidas ---------------------------------------------------------

    def publicar_tf(self, sello, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = sello
        t.header.frame_id = self.marco_odom
        t.child_frame_id = self.marco_base
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        qx, qy, qz, qw = cuaternion_de_yaw(yaw)
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.tf.sendTransform(t)

    def publicar_juntas(self, sello, posiciones):
        juntas = JointState()
        juntas.header.stamp = sello
        juntas.name = list(JUNTAS_CHASIS)
        juntas.position = [float(q) for q in posiciones]
        self.pub_juntas.publish(juntas)

    def publicar_verdad(self, sello, p, v_real, w_real):
        o = Odometry()
        o.header.stamp = sello
        o.header.frame_id = self.marco_odom
        o.child_frame_id = MARCO_CHASIS
        o.pose.pose.position.x = float(p.x)
        o.pose.pose.position.y = float(p.y)
        o.pose.pose.position.z = float(p.z)
        qx, qy, qz, qw = marcadores.cuaternion_zyx(p.yaw, p.cabeceo, p.balanceo)
        o.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        o.twist.twist.linear.x = float(v_real)
        o.twist.twist.angular.z = float(w_real)
        self.pub_verdad.publish(o)

    def publicar_imu(self, sello, p):
        # Con el robot real (espejo) la IMU es la suya: no inventarla
        if self.modo == 'espejo':
            return
        imu = Imu()
        imu.header.stamp = sello
        imu.header.frame_id = MARCO_IMU
        qx, qy, qz, qw = marcadores.cuaternion_zyx(p.yaw, p.cabeceo, p.balanceo)
        imu.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = 1e-6
        imu.orientation_covariance[8] = 1e-6
        imu.angular_velocity_covariance[0] = -1.0     # no se da
        g = 9.81
        imu.linear_acceleration.x = -g * math.sin(p.cabeceo)
        imu.linear_acceleration.y = g * math.cos(p.cabeceo) * math.sin(p.balanceo)
        imu.linear_acceleration.z = g * math.cos(p.cabeceo) * math.cos(p.balanceo)
        self.pub_imu.publish(imu)

    def publicar_estado(self, sello, v):
        e = self.estado
        p = e.pose
        z_suelo = self.mundo.altura(p.x, p.y)
        alto = None
        choque = marcadores.contacto_que_bloquea(e)
        dentro = marcadores.dentro_del_choque(choque) if choque is not None else None
        if dentro is not None:
            alto = self.mundo.altura(*dentro) - z_suelo
        avance_real = (self.ventana[0] / self.ventana[1]
                       if self.ventana[1] > 0.0 else 0.0)
        self.ventana = [0.0, 0.0]
        self.pub_estado.publish(String(data=marcadores.estado_json(
            e, mundo=self.titulo, version=self.version, modo=self.modo,
            z_suelo=z_suelo, avance_orugas=v, avance_real=avance_real,
            patinado=self.patinado, ms=self.ms, alto_choque=alto,
            aviso=self.aviso)))
        try:
            self.pub_indicadores.publish(marcadores.a_markers(
                marcadores.indicadores(e), self.marco_odom, sello,
                vida_s=marcadores.VIDA_INDICADORES_S))
        except ValueError as err:
            self.get_logger().error('Indicadores fuera de contrato: %s' % err,
                                    throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMundo()
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
