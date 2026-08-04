#!/usr/bin/env python3
"""
driver_node.py — puente ROS 2 <-> ESP32 de StarCrawler
======================================================
Unico nodo que habla con el hardware. Traduce entre topics ROS y el protocolo
serie de firmware/starcrawler_esp32_ros2 (ver protocol.py).

Suscribe:
    /cmd_vel            geometry_msgs/Twist          traccion (diferencial)
    /crawler/command    starcrawler_msgs/CrawlerCommand   orugas

Publica:
    /joint_states       sensor_msgs/JointState       -> robot_state_publisher
    /starcrawler/state  starcrawler_msgs/RobotState  estado completo
    /diagnostics        diagnostic_msgs/DiagnosticArray
    /odom + TF          nav_msgs/Odometry            (opcional, ver aviso)

Seguridad en capas:
  1. El ESP32 tiene su propio watchdog (300 ms) y para solo. Es la capa que
     manda: nunca se delega la seguridad a la red ni a ROS.
  2. Este nodo manda ceros si /cmd_vel o /crawler/command se quedan viejos.
  3. emergency_stop en CrawlerCommand llega al firmware como flag dedicado.
"""
from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from starcrawler_msgs.msg import CrawlerCommand, RobotState
from tf2_ros import TransformBroadcaster

from . import protocol as proto
from .simulator import Esp32Simulado

DEG = 180.0 / math.pi


class StarCrawlerDriver(Node):

    def __init__(self) -> None:
        super().__init__('starcrawler_driver')

        # ─── Parametros ──────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/starcrawler')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('simulate', False)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('tx_divider', 2)          # 100/2 = 50 Hz al ESP32
        self.declare_parameter('cmd_timeout_s', 0.5)
        # Geometria: SOLO afecta a la conversion cmd_vel <-> dps y a la odometria.
        # Calibrar midiendo: manda 0.05 m/s durante 10 s y mide lo recorrido.
        self.declare_parameter('wheel_radius', 0.025)     # radio efectivo motriz
        self.declare_parameter('track_separation', 0.40)  # entre ejes de oruga
        self.declare_parameter('max_track_speed_dps', 40.0)
        self.declare_parameter('joint_names', [
            'crawler_fr_joint', 'crawler_fl_joint',
            'crawler_rr_joint', 'crawler_rl_joint'])
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_odom', False)

        p = self.get_parameter
        self.simulate = p('simulate').value
        self.rate_hz = float(p('rate_hz').value)
        self.tx_divider = max(1, int(p('tx_divider').value))
        self.cmd_timeout = float(p('cmd_timeout_s').value)
        self.wheel_radius = float(p('wheel_radius').value)
        self.track_sep = float(p('track_separation').value)
        self.max_dps = float(p('max_track_speed_dps').value)
        self.joint_names = list(p('joint_names').value)
        self.base_frame = p('base_frame').value
        self.odom_frame = p('odom_frame').value
        self.publish_odom = bool(p('publish_odom').value)

        # ─── Estado interno ──────────────────────────────────────────────
        self.parser = proto.ParserEstado()
        self.puerto = None
        self.ultimo_estado: proto.Estado | None = None
        self.seq = 0
        self.ticks = 0

        self.cmd_vel = Twist()
        self.t_cmd_vel = 0.0
        self.crawler_cmd: CrawlerCommand | None = None
        self.t_crawler_cmd = 0.0

        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.t_odom = self.ahora()

        # ─── Interfaces ROS ──────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(
            CrawlerCommand, 'crawler/command', self.cb_crawler, 10)

        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_estado = self.create_publisher(RobotState, 'starcrawler/state', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10) \
            if self.publish_odom else None
        self.tf = TransformBroadcaster(self) if self.publish_odom else None

        self.abrir_puerto()
        self.create_timer(1.0 / self.rate_hz, self.bucle)
        self.create_timer(1.0, self.publicar_diagnostico)

        self.get_logger().info(
            f"StarCrawler driver listo ({'SIMULADO' if self.simulate else p('port').value})")

    # ─── Utilidades ──────────────────────────────────────────────────────

    def ahora(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def abrir_puerto(self) -> None:
        if self.simulate:
            self.puerto = Esp32Simulado()
            return
        try:
            import serial
        except ImportError:
            self.get_logger().error(
                'Falta pyserial: sudo apt install python3-serial')
            self.puerto = None
            return
        try:
            self.puerto = serial.Serial(
                self.get_parameter('port').value,
                self.get_parameter('baudrate').value,
                timeout=0)
        except Exception as e:  # noqa: BLE001 - queremos reintentar con cualquier fallo
            self.get_logger().warn(f'No se pudo abrir el puerto: {e}')
            self.puerto = None

    # ─── Callbacks ───────────────────────────────────────────────────────

    def cb_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.t_cmd_vel = self.ahora()

    def cb_crawler(self, msg: CrawlerCommand) -> None:
        self.crawler_cmd = msg
        self.t_crawler_cmd = self.ahora()

    # ─── Bucle principal ─────────────────────────────────────────────────

    def bucle(self) -> None:
        self.ticks += 1

        if self.puerto is None:
            if self.ticks % int(self.rate_hz * 2) == 0:
                self.abrir_puerto()
            return

        if self.simulate:
            self.puerto.avanzar(1.0 / self.rate_hz)

        if self.ticks % self.tx_divider == 0:
            self.enviar_comando()
        self.leer_estado()

    def enviar_comando(self) -> None:
        t = self.ahora()
        cmd = proto.Comando(seq=self.seq & 0xFF)
        self.seq += 1

        # Traccion: cinematica diferencial inversa
        if t - self.t_cmd_vel <= self.cmd_timeout:
            v = self.cmd_vel.linear.x
            w = self.cmd_vel.angular.z
            v_izq = v - w * self.track_sep / 2.0
            v_der = v + w * self.track_sep / 2.0
            if self.wheel_radius > 1e-6:
                dps_izq = (v_izq / self.wheel_radius) * DEG
                dps_der = (v_der / self.wheel_radius) * DEG
            else:
                dps_izq = dps_der = 0.0
            dps_izq = max(-self.max_dps, min(self.max_dps, dps_izq))
            dps_der = max(-self.max_dps, min(self.max_dps, dps_der))
            cmd.vel_izq_cdps = int(round(dps_izq * 100))
            cmd.vel_der_cdps = int(round(dps_der * 100))

        # Orugas
        c = self.crawler_cmd
        if c is not None and t - self.t_crawler_cmd <= self.cmd_timeout:
            cmd.emergencia = bool(c.emergency_stop)
            cmd.usar_posicion = bool(c.use_position)
            if c.use_position:
                cmd.objetivo_cdeg = [
                    int(round(proto.elevacion_a_enc(float(c.target[i]), i) * 100))
                    for i in range(proto.N_ORUGAS)]
                cmd.incremento = [0] * proto.N_ORUGAS
            else:
                cmd.incremento = [
                    proto.incremento_a_bruto(int(c.increment[i]), i)
                    for i in range(proto.N_ORUGAS)]

        try:
            self.puerto.write(proto.empaquetar_comando(cmd))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Escritura serie fallida: {e}')
            self.cerrar_puerto()

    def leer_estado(self) -> None:
        try:
            n = self.puerto.in_waiting
            datos = self.puerto.read(n) if n else b''
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Lectura serie fallida: {e}')
            self.cerrar_puerto()
            return

        for st in self.parser.alimentar(datos):
            self.ultimo_estado = st
            self.publicar_estado(st)

    def cerrar_puerto(self) -> None:
        try:
            if self.puerto is not None:
                self.puerto.close()
        except Exception:  # noqa: BLE001
            pass
        self.puerto = None

    # ─── Publicacion ─────────────────────────────────────────────────────

    def publicar_estado(self, st: proto.Estado) -> None:
        ahora = self.get_clock().now().to_msg()
        angulos = st.angulos_rad()
        oks = st.encoders_ok()

        js = JointState()
        js.header.stamp = ahora
        js.name = self.joint_names
        # El firmware solo refresca el angulo cuando la lectura I2C es valida,
        # asi que con un encoder averiado esto congela su ultimo valor bueno
        # (y encoder_ok en RobotState avisa de que no es de fiar).
        js.position = angulos
        self.pub_joints.publish(js)

        rs = RobotState()
        rs.header.stamp = ahora
        rs.header.frame_id = self.base_frame
        rs.crawler_angle = angulos
        rs.encoder_ok = oks
        rs.track_speed_left = math.radians(st.vel_izq_cdps / 100.0)
        rs.track_speed_right = math.radians(st.vel_der_cdps / 100.0)
        rs.can_ok = st.can_ok
        rs.imu_ok = st.imu_ok
        rs.safety_active = st.seguridad
        rs.error_bits = st.error_bits
        rs.frames_ok = self.parser.tramas_ok
        rs.frames_crc_error = self.parser.errores_crc
        self.pub_estado.publish(rs)

        if self.publish_odom:
            self.actualizar_odometria(st)

    def actualizar_odometria(self, st: proto.Estado) -> None:
        """AVISO: odometria de un vehiculo de orugas por velocidad de banda.
        En giro las orugas deslizan (skid steer), asi que el yaw acumula error
        rapido. Sirve como referencia local; para Nav2 fusionar con IMU/LiDAR.
        """
        t = self.ahora()
        dt = t - self.t_odom
        self.t_odom = t
        if dt <= 0.0 or dt > 0.5:
            return

        v_izq = math.radians(st.vel_izq_cdps / 100.0) * self.wheel_radius
        v_der = math.radians(st.vel_der_cdps / 100.0) * self.wheel_radius
        v = (v_izq + v_der) / 2.0
        w = (v_der - v_izq) / self.track_sep if self.track_sep > 1e-6 else 0.0

        self.odom_yaw += w * dt
        self.odom_x += v * math.cos(self.odom_yaw) * dt
        self.odom_y += v * math.sin(self.odom_yaw) * dt

        q = Quaternion()
        q.z = math.sin(self.odom_yaw / 2.0)
        q.w = math.cos(self.odom_yaw / 2.0)

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.odom_x
        od.pose.pose.position.y = self.odom_y
        od.pose.pose.orientation = q
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        self.pub_odom.publish(od)

        tr = TransformStamped()
        tr.header = od.header
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x = self.odom_x
        tr.transform.translation.y = self.odom_y
        tr.transform.rotation = q
        self.tf.sendTransform(tr)

    def publicar_diagnostico(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        enlace = DiagnosticStatus()
        enlace.name = 'starcrawler: enlace ESP32'
        enlace.hardware_id = str(self.get_parameter('port').value)
        if self.puerto is None:
            enlace.level = DiagnosticStatus.ERROR
            enlace.message = 'puerto serie cerrado'
        elif self.ultimo_estado is None:
            enlace.level = DiagnosticStatus.WARN
            enlace.message = 'sin telemetria todavia'
        else:
            enlace.level = DiagnosticStatus.OK
            enlace.message = 'simulado' if self.simulate else 'conectado'
        enlace.values = [
            KeyValue(key='tramas_ok', value=str(self.parser.tramas_ok)),
            KeyValue(key='errores_crc', value=str(self.parser.errores_crc)),
            KeyValue(key='bytes_descartados',
                     value=str(self.parser.bytes_descartados)),
        ]
        arr.status.append(enlace)

        st = self.ultimo_estado
        if st is not None:
            hw = DiagnosticStatus()
            hw.name = 'starcrawler: hardware'
            hw.hardware_id = 'esp32'
            fallos = []
            for i, nombre in enumerate(proto.ORUGAS):
                if (st.error_bits >> i) & 1:
                    fallos.append(f'encoder {nombre}')
            if not st.can_ok:
                fallos.append('bus CAN')
            hw.level = DiagnosticStatus.ERROR if fallos else DiagnosticStatus.OK
            hw.message = ', '.join(fallos) if fallos else 'todo OK'
            hw.values = [
                KeyValue(key='error_bits', value=f'0x{st.error_bits:04X}'),
                KeyValue(key='seguridad', value=str(st.seguridad)),
            ] + [
                KeyValue(key=f'angulo_{n}', value=f'{st.angulo_cdeg[i] / 100.0:.1f} deg')
                for i, n in enumerate(proto.ORUGAS)
            ]
            arr.status.append(hw)

        self.pub_diag.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = StarCrawlerDriver()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar_puerto()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
