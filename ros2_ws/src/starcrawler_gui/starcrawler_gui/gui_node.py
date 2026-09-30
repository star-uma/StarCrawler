"""
gui_node.py — el dashboard leyendo de ROS 2
===========================================
El dashboard de la rama standalone (dashboard.py, copiado sin tocar) ya
tenia las fuentes de datos desacopladas del dibujo: serie, UDP y demo.
Esto anade una cuarta fuente, /starcrawler/state, y reutiliza tal cual la
pagina, el dibujo del robot y las graficas.

    ros2 run starcrawler_gui gui_node
    -> http://localhost:8000      telemetria 2D
    -> http://localhost:8000/3d   el robot moviendose por el plano

Durante la puesta en marcha vale mas que leer numeros: un encoder cruzado
o un signo invertido se ven de un vistazo en las cuatro orugas dibujadas.

La vista 3D (vista3d.py) necesita ademas /odom, /joint_states y el URDF de
/robot_description, que se sirve traducido en /modelo.

Con un mundo simulado (mundo:=, mundo_node) lee tambien /mundo/*: la
geometria se sirve traducida en /mundo (mundo_modelo.py) y por /events solo
van la version, la pose verdadera, el estado del terreno y los indicadores.

Con mando:=true la pagina /3d tambien conduce (issue #18): publica en
cmd_vel_web y crawler/command_web, que entran a los muxes. Las rutas HTTP
estan en servidor.py y la logica en mando_web.py; este nodo solo publica,
y siempre desde su temporizador.
"""
from __future__ import annotations

import json
import math
import os
import re
import secrets
import socket
import threading
import time
import webbrowser

import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from visualization_msgs.msg import MarkerArray

from starcrawler_msgs.msg import CrawlerCommand, RobotState

from starcrawler_common import orugas
from starcrawler_common.angulos import elevacion_a_encoder_deg

from . import dashboard
from .dashboard import registrar
from .mando_web import PRESET_MAX_S, Arbitro
from .mundo_modelo import Traductor, traducir_lista, validar_estado
from .servidor import (MODELO, MUNDO, THREE_LOCAL, crear_manejador,
                       crear_servidor)
from .urdf_modelo import leer_urdf

RAD_A_GRADOS = 57.29577951
PERIODO_MANDO_S = 0.02      # 50 Hz
ENLACE_S = 1.0              # el mismo umbral que el indicador de dashboard.py
AVISO_TELEOP_S = 5.0
PERIODO_CADUCA_S = 0.2      # 5 Hz
CADUCA_S = 0.5              # terreno, marcas y verdad sin llegar -> None


def ip_local() -> str:
    """La IP con la que este equipo sale a la red. No envia nada."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'


class NodoDashboard(Node):

    def __init__(self):
        super().__init__('starcrawler_gui')

        self.declare_parameter('http_port', 8000)
        # En el PC de a bordo no hay nadie delante de la pantalla, asi que
        # por defecto no se abre nada: se entra desde otro equipo.
        self.declare_parameter('abrir_navegador', False)
        # Conducir desde /3d. Sin el, los POST responden 403
        self.declare_parameter('mando', False)
        # Vacia = una al azar en cada arranque
        self.declare_parameter('mando_token', '')
        # El launch pasa los de ds4.yaml: la web y el DS4 con los mismos topes
        self.declare_parameter('max_lineal', orugas.MAX_LINEAL)
        self.declare_parameter('max_angular', orugas.MAX_ANGULAR)
        self.declare_parameter('factor_lento', orugas.FACTOR_LENTO)
        self.declare_parameter('s_preset', orugas.S_PRESET)
        self.declare_parameter('preset_max_s', PRESET_MAX_S)
        # Marco de los marcadores del mundo; los de otro marco no se dibujan
        self.declare_parameter('marco_fijo', 'odom')

        puerto = self.get_parameter('http_port').value
        self.mando = bool(self.get_parameter('mando').value)
        self.marco_fijo = str(self.get_parameter('marco_fijo').value)

        # Lo que el arbitro necesita del robot y de los muxes. Se escribe y
        # se lee con lock_mando, que tambien protege al arbitro.
        self.lock_mando = threading.Lock()
        self._elev = [0.0] * 4
        self._encoder_ok = [False] * 4
        self._t_estado = None
        self._activa = ''
        self._mux_ok = False
        self._rearme = False
        self.arbitro = None
        with dashboard.LOCK:
            dashboard.ESTADO['seguridad'] = False
            dashboard.ESTADO['mando'] = {'habilitado': False}

        # El ESP32 publica en best-effort: una suscripcion fiable no casa
        self.create_subscription(
            RobotState, 'starcrawler/state', self.cb_estado,
            qos_profile_sensor_data)

        # Para la vista 3D
        self.create_subscription(
            Odometry, 'odom', self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(
            JointState, 'joint_states', self.cb_juntas,
            qos_profile_sensor_data)
        # robot_state_publisher lo publica una vez, latcheado
        self.create_subscription(
            String, 'robot_description', self.cb_descripcion,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.preparar_mundo()

        clave = self.preparar_mando() if self.mando else ''

        manejador = crear_manejador(
            arbitro=self.arbitro, lock=self.lock_mando, clave=clave,
            mando=self.mando, contexto=self.contexto,
            rearmar=self.pedir_rearme, reloj=time.monotonic)
        self.servidor = crear_servidor(puerto, manejador)
        threading.Thread(target=self.servidor.serve_forever,
                         kwargs={'poll_interval': 0.1}, daemon=True).start()

        url = 'http://localhost:%d' % puerto
        self.get_logger().info('Dashboard en %s (vista 3D en %s/3d)'
                               % (url, url))
        if self.mando:
            self.get_logger().info('Mando web ACTIVO: http://%s:%d/3d#t=%s'
                                   % (ip_local(), puerto, clave))
        if self.get_parameter('abrir_navegador').value:
            webbrowser.open(url)

    # ─── Mando web ───────────────────────────────────────────────────────

    def preparar_mando(self) -> str:
        p = self.get_parameter
        self.arbitro = Arbitro(
            max_lineal=float(p('max_lineal').value),
            max_angular=float(p('max_angular').value),
            factor_lento=float(p('factor_lento').value),
            s_preset=float(p('s_preset').value),
            preset_max_s=float(p('preset_max_s').value))

        self.pub_vel = self.create_publisher(Twist, 'cmd_vel_web', 10)
        self.pub_orugas = self.create_publisher(
            CrawlerCommand, 'crawler/command_web', 10)
        self.pub_rearmar = self.create_publisher(
            Empty, 'crawler_mux/rearmar', 10)
        # crawler_mux la publica latcheada y solo al cambiar
        self.create_subscription(
            String, 'crawler_mux/activa', self.cb_activa,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(PERIODO_MANDO_S, self.cb_mando)
        self._aviso_teleop = self.create_timer(AVISO_TELEOP_S,
                                               self.comprobar_teleop)

        if not os.path.exists(THREE_LOCAL):
            self.get_logger().warn(
                'Sin copia local de three.js: sin internet /3d no dibuja el '
                'robot (ver static/README.md)')
        clave = str(p('mando_token').value)
        # Va en la URL tal cual: con &, % o espacios la pagina no la leeria
        if clave and not re.fullmatch(r'[A-Za-z0-9_-]{4,64}', clave):
            self.get_logger().error(
                'gui_clave solo admite letras, cifras, _ y - (de 4 a 64): '
                'uso una al azar')
            clave = ''
        return clave or secrets.token_hex(5)

    def comprobar_teleop(self):
        self._aviso_teleop.cancel()
        if 'starcrawler_teleop' not in self.get_node_names():
            self.get_logger().warn(
                'Mando web sin teleop: no hay parada fisica (SHARE), solo '
                'la EMERGENCIA de la pagina')

    def contexto(self):
        """Para los hilos HTTP, que lo llaman con lock_mando cogido."""
        return self._enlace(time.monotonic()), self._activa, self._mux_ok

    def pedir_rearme(self):
        # Tambien con lock_mando: el Empty lo publica cb_mando
        self._rearme = True

    def _enlace(self, t: float) -> bool:
        return self._t_estado is not None and t - self._t_estado < ENLACE_S

    def cb_activa(self, msg: String):
        with self.lock_mando:
            self._activa = msg.data

    def cb_mando(self):
        t = time.monotonic()
        mux_ok = (self.pub_vel.get_subscription_count() > 0
                  and self.pub_orugas.get_subscription_count() > 0)
        with self.lock_mando:
            self._mux_ok = mux_ok
            robot = {'elev': self._elev, 'encoder_ok': self._encoder_ok,
                     'enlace': self._enlace(t)}
            c = self.arbitro.salida(t, robot)
            estado = self.arbitro.estado(t, self._activa, mux_ok)
            eventos = self.arbitro.sacar_eventos()
            rearme, self._rearme = self._rearme, False
        with dashboard.LOCK:
            dashboard.ESTADO['mando'] = estado

        for e in eventos:
            self.get_logger().info(e)
        if rearme:
            self.get_logger().info('Rearme pedido desde la web')
            self.pub_rearmar.publish(Empty())
        if c is None:
            return

        vel = Twist()
        vel.linear.x = float(c.lineal)
        vel.angular.z = float(c.angular)
        cmd = CrawlerCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.increment = [int(v) for v in c.incremento]
        cmd.use_position = bool(c.usar_posicion)
        cmd.target = [float(v) for v in c.objetivo_rad]
        cmd.emergency_stop = bool(c.emergencia)
        self.pub_vel.publish(vel)
        self.pub_orugas.publish(cmd)

    # ─── Telemetria ──────────────────────────────────────────────────────

    def cb_estado(self, msg: RobotState):
        angulos = [elevacion_a_encoder_deg(i, msg.crawler_angle[i])
                   for i in range(4)]

        with self.lock_mando:
            self._elev = [float(a) for a in msg.crawler_angle]
            self._encoder_ok = [bool(v) for v in msg.encoder_ok]
            self._t_estado = time.monotonic()

        registrar(
            fuente='ros2',
            ang=angulos,
            vl=msg.track_speed_left * RAD_A_GRADOS,
            vr=msg.track_speed_right * RAD_A_GRADOS,
            err=int(msg.error_bits),
            # Sin modos: en ROS 2 la traccion y la elevacion conviven, no
            # hay ciclado de modos como en el firmware del TFG.
            modo=0,
            con_imu=bool(msg.imu_ok),
            roll=None,
            pitch=None,
            seguridad=bool(msg.safety_active),
        )

    # Estos dos no pasan por registrar(): no son tramas del robot y no
    # deben contar para los paquetes/s ni para el indicador de enlace.
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with dashboard.LOCK:
            dashboard.ESTADO['pose'] = [
                p.x, p.y, yaw,
                msg.twist.twist.linear.x, msg.twist.twist.angular.z]

    def cb_juntas(self, msg: JointState):
        with dashboard.LOCK:
            juntas = dict(dashboard.ESTADO.get('joints') or {})
            juntas.update(zip(msg.name, msg.position))
            dashboard.ESTADO['joints'] = juntas

    def cb_descripcion(self, msg: String):
        try:
            MODELO['json'] = json.dumps(leer_urdf(msg.data))
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.get_logger().error('No puedo leer /robot_description: %s' % e)
            return
        self.get_logger().info('Modelo del robot cargado para la vista 3D')

    # ─── Mundo simulado (mundo_node) ─────────────────────────────────────
    # Tampoco pasan por registrar(). La geometria va a MUNDO (GET /mundo),
    # nunca a /events.

    def preparar_mundo(self):
        self.traductor = Traductor(self.marco_fijo)
        self._t_mundo = {'terreno': None, 'marcas': None, 'verdad': None}
        with dashboard.LOCK:
            dashboard.ESTADO['mundo_v'] = 0
            for clave in self._t_mundo:
                dashboard.ESTADO[clave] = None

        self.create_subscription(
            MarkerArray, 'mundo/marcadores', self.cb_marcadores,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            MarkerArray, 'mundo/indicadores', self.cb_indicadores,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            String, 'mundo/estado', self.cb_terreno,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Odometry, 'mundo/verdad', self.cb_verdad, qos_profile_sensor_data)
        self.create_timer(PERIODO_CADUCA_S, self.cb_caducar)

    def _guardar(self, clave, valor):
        with dashboard.LOCK:
            dashboard.ESTADO[clave] = valor
            self._t_mundo[clave] = time.monotonic()

    def cb_marcadores(self, msg: MarkerArray):
        try:
            self.traductor.aplicar(msg.markers)
            datos = self.traductor.a_json()
            texto = json.dumps(datos)
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.get_logger().error('No puedo leer /mundo/marcadores: %s' % e)
            return
        # Primero el JSON: quien vea la version nueva ya lo encuentra
        MUNDO['json'] = texto
        MUNDO['version'] = datos['version']
        with dashboard.LOCK:
            dashboard.ESTADO['mundo_v'] = datos['version']
        self.get_logger().info(
            'Mundo para la vista 3D: %d piezas (version %d)'
            % (len(datos['piezas']), datos['version']))
        if datos['omitidos']:
            self.get_logger().warn(
                '%d marcadores del mundo sin dibujar en la web (tipo, marco '
                'distinto de %s o numeros no finitos)'
                % (datos['omitidos'], self.marco_fijo))

    def cb_indicadores(self, msg: MarkerArray):
        try:
            marcas = traducir_lista(msg.markers, self.marco_fijo)
        except Exception as e:
            self.get_logger().warn('/mundo/indicadores ignorado: %s' % e,
                                   throttle_duration_sec=5.0)
            return
        self._guardar('marcas', marcas)

    def cb_terreno(self, msg: String):
        try:
            terreno = validar_estado(msg.data)
        except ValueError as e:
            self.get_logger().warn('/mundo/estado ignorado: %s' % e,
                                   throttle_duration_sec=5.0)
            return
        self._guardar('terreno', terreno)

    def cb_verdad(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # La misma formula ZYX que cb_odom
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        verdad = [p.x, p.y, yaw, msg.twist.twist.linear.x,
                  msg.twist.twist.angular.z, p.z]
        if not all(math.isfinite(v) for v in verdad):
            self.get_logger().warn('/mundo/verdad con numeros no finitos',
                                   throttle_duration_sec=5.0)
            return
        self._guardar('verdad', verdad)

    def cb_caducar(self):
        t = time.monotonic()
        with dashboard.LOCK:
            for clave, t0 in self._t_mundo.items():
                if t0 is not None and t - t0 > CADUCA_S:
                    dashboard.ESTADO[clave] = None
                    self._t_mundo[clave] = None

    def destroy_node(self):
        self.servidor.shutdown()
        self.servidor.server_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoDashboard()
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
