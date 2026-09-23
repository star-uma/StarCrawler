"""
gui_node.py — el dashboard leyendo de ROS 2
===========================================
El dashboard de la rama standalone (dashboard.py, copiado sin tocar) ya
tenia las fuentes de datos desacopladas del dibujo: serie, UDP y demo.
Esto anade una cuarta fuente, /starcrawler/state, y reutiliza tal cual la
pagina, el dibujo del robot y las graficas.

    ros2 run starcrawler_gui gui_node
    -> http://localhost:8000

Durante la puesta en marcha vale mas que leer numeros: un encoder cruzado
o un signo invertido se ven de un vistazo en las cuatro orugas dibujadas.
"""
from __future__ import annotations

import threading
import webbrowser
from http.server import ThreadingHTTPServer

import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from starcrawler_msgs.msg import RobotState

from starcrawler_common.angulos import elevacion_a_encoder_deg

from .dashboard import Manejador, registrar

RAD_A_GRADOS = 57.29577951


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

        self.servidor = ThreadingHTTPServer(('', puerto), Manejador)
        threading.Thread(target=self.servidor.serve_forever,
                         kwargs={'poll_interval': 0.1}, daemon=True).start()

        url = 'http://localhost:%d' % puerto
        self.get_logger().info('Dashboard en %s' % url)
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
