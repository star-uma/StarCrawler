"""Publica /joy a partir de datagramas UDP del puente de Windows.

WSL no ve el mando (ni por Bluetooth ni por USB), asi que un script en
Windows lo lee con pygame y lo envia aqui. El puente ya entrega los ejes y
botones en el mismo orden y signo que el nodo `joy` de Linux con un
DualShock 4: el teleop y ds4.yaml no distinguen una fuente de otra.
Ver tools/joy_bridge/README.md.

Formato del datagrama (JSON): {"axes": [..8 floats..], "buttons": [..13 ints..]}
"""
import json
import signal
import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class NodoJoyUdp(Node):

    def __init__(self):
        super().__init__('joy_udp_node')
        self.declare_parameter('puerto', 8890)
        # Sin datagramas en este tiempo se publica un mensaje neutro, para
        # que el teleop no se quede con la ultima consigna del mando.
        self.declare_parameter('timeout_ms', 500)
        puerto = self.get_parameter('puerto').value
        self.timeout_ms = self.get_parameter('timeout_ms').value

        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', puerto))
        self.sock.setblocking(False)

        self.ultimo = None
        self.ms_sin_datos = 0
        self.avisado = False
        self.create_timer(0.01, self.tick)
        self.get_logger().info('Esperando al puente del mando en UDP %d' % puerto)

    def tick(self):
        datos = None
        try:
            while True:      # nos quedamos con el datagrama mas reciente
                datos, _ = self.sock.recvfrom(1024)
        except BlockingIOError:
            pass

        if datos is not None:
            try:
                d = json.loads(datos.decode())
                msg = Joy()
                msg.axes = [float(v) for v in d['axes']]
                msg.buttons = [int(v) for v in d['buttons']]
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning('Datagrama no valido, lo ignoro')
                return
            if self.ultimo is None or self.avisado:
                self.get_logger().info('Puente del mando conectado')
                self.avisado = False
            self.ultimo = msg
            self.ms_sin_datos = 0
        elif self.ultimo is not None:
            self.ms_sin_datos += 10
            if self.ms_sin_datos >= self.timeout_ms:
                if not self.avisado:
                    self.get_logger().warning('Puente del mando perdido: mando neutro')
                    self.avisado = True
                msg = Joy()
                msg.axes = [0.0] * len(self.ultimo.axes)
                msg.buttons = [0] * len(self.ultimo.buttons)
                self.ultimo = msg
            else:
                return
        else:
            return

        msg = self.ultimo
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoJoyUdp()
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
