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
"""
from __future__ import annotations

import json
import math
import os
import threading
import webbrowser
from http.server import ThreadingHTTPServer

import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from starcrawler_msgs.msg import RobotState

from starcrawler_common.angulos import elevacion_a_encoder_deg

from . import dashboard
from .dashboard import Manejador, registrar
from .urdf_modelo import leer_urdf
from .vista3d import THREE_CDN, enlazar_desde_2d, pagina_3d

RAD_A_GRADOS = 57.29577951

# Copia local de three.js para usar la vista 3D sin internet (opcional)
THREE_LOCAL = os.path.join(os.path.dirname(__file__), 'static',
                           'three.module.min.js')

# Lo escribe el nodo al llegar /robot_description y lo lee el servidor HTTP
_MODELO = {'json': None}


class ManejadorRos(Manejador):
    """El de dashboard.py mas las rutas de la vista 3D."""

    def do_GET(self):
        if self.path == '/':
            self._enviar(enlazar_desde_2d(dashboard.PAGINA),
                         'text/html; charset=utf-8')
        elif self.path == '/3d':
            local = os.path.exists(THREE_LOCAL)
            url = '/static/three.module.min.js' if local else THREE_CDN
            self._enviar(pagina_3d(url), 'text/html; charset=utf-8')
        elif self.path == '/modelo':
            modelo = _MODELO['json']
            if modelo is None:
                self.send_error(503, 'Sin /robot_description todavia')
            else:
                self._enviar(modelo, 'application/json')
        elif (self.path == '/static/three.module.min.js'
              and os.path.exists(THREE_LOCAL)):
            with open(THREE_LOCAL, 'rb') as f:
                self._enviar(f.read(), 'text/javascript')
        else:
            super().do_GET()

    def _enviar(self, cuerpo, tipo):
        if isinstance(cuerpo, str):
            cuerpo = cuerpo.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', tipo)
        self.send_header('Content-Length', str(len(cuerpo)))
        self.end_headers()
        self.wfile.write(cuerpo)


class NodoDashboard(Node):

    def __init__(self):
        super().__init__('starcrawler_gui')

        self.declare_parameter('http_port', 8000)
        # En el PC de a bordo no hay nadie delante de la pantalla, asi que
        # por defecto no se abre nada: se entra desde otro equipo.
        self.declare_parameter('abrir_navegador', False)

        puerto = self.get_parameter('http_port').value

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

        self.servidor = ThreadingHTTPServer(('', puerto), ManejadorRos)
        threading.Thread(target=self.servidor.serve_forever,
                         kwargs={'poll_interval': 0.1}, daemon=True).start()

        url = 'http://localhost:%d' % puerto
        self.get_logger().info('Dashboard en %s (vista 3D en %s/3d)'
                               % (url, url))
        if self.get_parameter('abrir_navegador').value:
            webbrowser.open(url)

    def cb_estado(self, msg: RobotState):
        angulos = [elevacion_a_encoder_deg(i, msg.crawler_angle[i])
                   for i in range(4)]

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
            _MODELO['json'] = json.dumps(leer_urdf(msg.data))
        except Exception as e:  # viene de fuera: no tumbar el nodo
            self.get_logger().error('No puedo leer /robot_description: %s' % e)
            return
        self.get_logger().info('Modelo del robot cargado para la vista 3D')

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
