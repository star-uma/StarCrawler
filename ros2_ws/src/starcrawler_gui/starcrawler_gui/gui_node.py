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

import rclpy
from rclpy.node import Node

from starcrawler_msgs.msg import RobotState

from .dashboard import Manejador, registrar

RAD_A_GRADOS = 57.29577951


def es_espejada(i: int) -> bool:
    """FL y RR van espejadas, como documenta el TFG."""
    return i == 1 or i == 2


def elevacion_a_encoder_deg(i: int, elevacion_rad: float) -> float:
    """Radianes de elevacion -> grados de encoder.

    Inverso de lo que hace el firmware. El dashboard dibuja en grados de
    encoder (180 = oruga horizontal), asi que hay que deshacer la
    conversion para no tener que tocar el dibujo.
    """
    d = elevacion_rad * RAD_A_GRADOS
    return (180.0 + d) if es_espejada(i) else (180.0 - d)


class NodoDashboard(Node):

    def __init__(self):
        super().__init__('starcrawler_gui')

        self.declare_parameter('http_port', 8000)
        # En el PC de a bordo no hay nadie delante de la pantalla, asi que
        # por defecto no se abre nada: se entra desde otro equipo.
        self.declare_parameter('abrir_navegador', False)

        puerto = self.get_parameter('http_port').value

        self.create_subscription(
            RobotState, 'starcrawler/state', self.cb_estado, 10)

        self.servidor = ThreadingHTTPServer(('', puerto), Manejador)
        threading.Thread(target=self.servidor.serve_forever,
                         daemon=True).start()

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
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoDashboard()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
