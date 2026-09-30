"""
servidor.py — las rutas HTTP de gui_node, sin ROS
=================================================
GET: la telemetria 2D y /events (de dashboard.py), /3d, /modelo, /mundo y
la copia local de three.js. POST: /mando, /emergencia y /rearmar, que solo
responden con mando:=true.

Lo que viene del nodo llega como atributos de clase (crear_manejador): el
Arbitro, su Lock, la clave y dos funciones. Los hilos HTTP solo validan y
guardan; publica el temporizador de gui_node. Sin do_OPTIONS ni cabeceras
CORS: el preflight de una web ajena recibe 501.
"""
from __future__ import annotations

import json
import os
import socket
import threading
import time
from http.server import ThreadingHTTPServer

from . import dashboard
from .dashboard import Manejador
from .mando_web import CABECERA_CLAVE, RUTAS_POST  # noqa: F401 (la pagina)
from .mando_web import (
    MAX_CUERPO,
    comprobar_cabeceras,
    longitud_cuerpo,
    validar_mando,
)
from .vista3d import THREE_CDN, enlazar_desde_2d, pagina_3d

# Copia local de three.js para usar la vista 3D sin internet (opcional)
THREE_LOCAL = os.path.join(os.path.dirname(__file__), 'static',
                           'three.module.min.js')

# Lo escribe el nodo al llegar /robot_description y lo lee el servidor HTTP
MODELO = {'json': None}
# Igual con /mundo/marcadores. Sin mundo, /mundo responde MUNDO_VACIO
MUNDO = {'json': None, 'version': 0}
MUNDO_VACIO = json.dumps({'version': 0, 'piezas': [], 'omitidos': 0})


class ManejadorRos(Manejador):
    """El de dashboard.py mas la vista 3D y el mando."""

    timeout = 5

    arbitro = None
    lock = threading.Lock()
    clave = ''
    mando = False
    # () -> (enlace con el ESP32, crawler_mux/activa, mux_ok)
    contexto = staticmethod(lambda: (False, '', False))
    rearmar = staticmethod(lambda: None)
    reloj = staticmethod(time.monotonic)

    def do_GET(self):
        if self.path == '/':
            self._enviar(enlazar_desde_2d(dashboard.PAGINA),
                         'text/html; charset=utf-8')
        elif self.path == '/3d':
            local = os.path.exists(THREE_LOCAL)
            url = '/static/three.module.min.js' if local else THREE_CDN
            self._enviar(pagina_3d(url), 'text/html; charset=utf-8')
        elif self.path == '/modelo':
            modelo = MODELO['json']
            if modelo is None:
                self.send_error(503, 'Sin /robot_description todavia')
            else:
                self._enviar(modelo, 'application/json')
        elif self.path == '/mundo':
            # No tener mundo es lo normal: siempre 200
            cuerpo = (MUNDO['json'] or MUNDO_VACIO).encode('utf-8')
            self.connection.settimeout(60)      # puede pasar de 1 MB
            self.send_response(200)
            self.send_header('Cache-Control', 'no-store')
            self._cabeceras_y_cuerpo(cuerpo, 'application/json')
        elif (self.path == '/static/three.module.min.js'
              and os.path.exists(THREE_LOCAL)):
            # timeout vale para cada operacion: 5 s no dan para 700 KB en una
            # WiFi floja
            self.connection.settimeout(60)
            with open(THREE_LOCAL, 'rb') as f:
                self._enviar(f.read(), 'text/javascript')
        else:
            super().do_GET()

    def setup(self):
        super().setup()
        # HTTP/1.0: una conexion por POST, asi que es la hora de la peticion
        self._t_conexion = self.reloj()

    def do_POST(self):
        ruta = self.path
        if ruta == '/emergencia' and self.mando:
            # Parar no espera al cuerpo ni se niega por una formalidad
            with self.lock:
                self.arbitro.emergencia(self.reloj(), self._ip())
            self._responder(202, 'emergencia pedida')
            self._descartar_cuerpo()
            return
        fallo = comprobar_cabeceras(ruta, self.headers, self.mando, self.clave)
        if fallo is not None:
            self._responder(*fallo)
            self._descartar_cuerpo()
            return
        n = longitud_cuerpo(self.headers) or 0
        cuerpo = self.rfile.read(n) if n else b''
        if len(cuerpo) != n:
            self._responder(400, 'cuerpo incompleto')
            return

        if ruta == '/mando':
            try:
                orden = validar_mando(cuerpo)
            except ValueError as e:
                self._responder(400, str(e))
                return
            with self.lock:
                enlace, activa, mux_ok = self.contexto()
                codigo, motivo = self.arbitro.mando(
                    orden, self._ip(), self.reloj(),
                    enlace, activa, mux_ok, self._t_conexion)
            self._responder(codigo, motivo)
        else:
            with self.lock:
                self.rearmar()
            self._responder(202, 'rearme pedido')

    def _responder(self, codigo, motivo):
        cuerpo = json.dumps({'ok': codigo < 300, 'motivo': motivo})
        self.send_response(codigo)
        self.send_header('Cache-Control', 'no-store')
        self._cabeceras_y_cuerpo(cuerpo.encode('utf-8'), 'application/json')

    def _descartar_cuerpo(self):
        # Tras un rechazo temprano: si el cliente ya mando el cuerpo, cerrar
        # sin leerlo puede llegarle como RST en vez de la respuesta
        try:
            n = longitud_cuerpo(self.headers)
        except ValueError:
            return
        if not n or n > MAX_CUERPO:
            return
        try:
            self.connection.settimeout(0.2)
            self.rfile.read(n)
        except OSError:
            pass

    def _enviar(self, cuerpo, tipo):
        if isinstance(cuerpo, str):
            cuerpo = cuerpo.encode('utf-8')
        self.send_response(200)
        self._cabeceras_y_cuerpo(cuerpo, tipo)

    def _ip(self):
        # En doble pila los clientes IPv4 llegan como ::ffff:a.b.c.d
        ip = self.client_address[0]
        return ip[7:] if ip.startswith('::ffff:') else ip

    def _cabeceras_y_cuerpo(self, cuerpo, tipo):
        self.send_header('Content-Type', tipo)
        self.send_header('Content-Length', str(len(cuerpo)))
        self.end_headers()
        self.wfile.write(cuerpo)


def crear_manejador(arbitro=None, lock=None, clave='', mando=False,
                    contexto=None, rearmar=None, reloj=None):
    """Una subclase de ManejadorRos con lo que le da el nodo."""
    if mando and arbitro is None:
        raise ValueError('con mando hace falta un Arbitro')
    atributos = {'arbitro': arbitro, 'lock': lock or threading.Lock(),
                 'clave': clave, 'mando': bool(mando)}
    for nombre, f in (('contexto', contexto), ('rearmar', rearmar),
                      ('reloj', reloj)):
        if f is not None:
            atributos[nombre] = staticmethod(f)
    return type('ManejadorMando', (ManejadorRos,), atributos)


class ServidorDoble(ThreadingHTTPServer):
    """IPv6 e IPv4 en el mismo puerto. Para localhost el navegador prueba
    antes ::1 y, si ahi no hay nadie, espera 300 ms para ir a IPv4: con el
    reenvio de localhost del WSL cada POST del mando llegaba tarde."""
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


def crear_servidor(puerto, manejador):
    try:
        return ServidorDoble(('::', puerto), manejador)
    except OSError:          # maquina sin IPv6
        return ThreadingHTTPServer(('', puerto), manejador)
