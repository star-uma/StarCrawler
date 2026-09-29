"""Tests del servidor HTTP con un servidor de verdad en 127.0.0.1. Sin ROS."""
import http.client
import json
import socket
import threading
import time
from http.server import ThreadingHTTPServer

import pytest

from starcrawler_gui.mando_web import CABECERA_CLAVE, Arbitro
from starcrawler_gui.servidor import RUTAS_POST, crear_manejador

CLAVE = 'a1b2c3d4e5'
ORDEN = {'sesion': 'sesionA1', 'seq': 0, 'avance': 0.5, 'giro': 0.0,
         'lento': False, 'delanteras': 0, 'traseras': 0,
         'inclinar': None, 'preset': None}
ROBOT = {'elev': [0.0] * 4, 'encoder_ok': [True] * 4, 'enlace': True}


class Nodo:
    """Lo que gui_node le da al servidor."""

    def __init__(self):
        self.arbitro = Arbitro()
        self.lock = threading.Lock()
        self.rearmes = 0
        self.ctx = (True, 'web', True)

    def contexto(self):
        return self.ctx

    def rearmar(self):
        self.rearmes += 1


@pytest.fixture
def arrancar():
    servidores = []

    def _arrancar(mando=True, nodo=None):
        nodo = nodo or Nodo()
        manejador = crear_manejador(
            arbitro=nodo.arbitro, lock=nodo.lock, clave=CLAVE, mando=mando,
            contexto=nodo.contexto, rearmar=nodo.rearmar)
        srv = ThreadingHTTPServer(('127.0.0.1', 0), manejador)
        threading.Thread(target=srv.serve_forever,
                         kwargs={'poll_interval': 0.05}, daemon=True).start()
        servidores.append(srv)
        return srv.server_address[1], nodo

    yield _arrancar
    for srv in servidores:
        srv.shutdown()
        srv.server_close()


def post(puerto, ruta, cuerpo=b'', clave=CLAVE, tipo='application/json'):
    cab = {}
    if clave is not None:
        cab[CABECERA_CLAVE] = clave
    if tipo is not None:
        cab['Content-Type'] = tipo
    c = http.client.HTTPConnection('127.0.0.1', puerto, timeout=3)
    try:
        c.request('POST', ruta, body=cuerpo, headers=cab)
        r = c.getresponse()
        datos = r.read()
        return r.status, (json.loads(datos) if datos else None), r
    finally:
        c.close()


def orden(**cambios):
    d = dict(ORDEN)
    d.update(cambios)
    return json.dumps(d).encode('utf-8')


def test_las_rutas_post_son_tres():
    assert RUTAS_POST == ('/mando', '/emergencia', '/rearmar')


def test_options_es_501(arrancar):
    puerto, _ = arrancar()
    c = http.client.HTTPConnection('127.0.0.1', puerto, timeout=3)
    c.request('OPTIONS', '/mando', headers={
        'Origin': 'http://otra.web',
        'Access-Control-Request-Method': 'POST',
        'Access-Control-Request-Headers': CABECERA_CLAVE})
    r = c.getresponse()
    assert r.status == 501
    assert r.getheader('Access-Control-Allow-Origin') is None
    c.close()


def test_sin_clave_401_sin_esperar_al_cuerpo(arrancar):
    """Solo cabeceras, anunciando un cuerpo que nunca llega."""
    puerto, nodo = arrancar()
    s = socket.create_connection(('127.0.0.1', puerto), timeout=2)
    try:
        t0 = time.monotonic()
        s.sendall(b'POST /mando HTTP/1.1\r\nHost: x\r\n'
                  b'Content-Type: application/json\r\n'
                  b'Content-Length: 200\r\n\r\n')
        respuesta = s.recv(4096)
        assert respuesta.startswith(b'HTTP/1.0 401')
        assert time.monotonic() - t0 < 1.0
    finally:
        s.close()
    assert nodo.arbitro.salida(time.monotonic(), ROBOT) is None


def test_clave_mala_401(arrancar):
    puerto, _ = arrancar()
    estado, datos, _ = post(puerto, '/mando', orden(), clave='mala')
    assert estado == 401 and datos['ok'] is False


def test_text_plain_415(arrancar):
    puerto, _ = arrancar()
    estado, datos, _ = post(puerto, '/mando', orden(), tipo='text/plain')
    assert estado == 415 and set(datos) == {'ok', 'motivo'}


def test_sin_mando_403(arrancar):
    puerto, _ = arrancar(mando=False)
    for ruta in RUTAS_POST:
        assert post(puerto, ruta, orden())[0] == 403


def test_cuerpo_grande_413(arrancar):
    puerto, _ = arrancar()
    assert post(puerto, '/mando', b' ' * 2000)[0] == 413


def test_json_malo_400(arrancar):
    puerto, nodo = arrancar()
    estado, datos, _ = post(puerto, '/mando', orden(avance=float('nan')))
    assert estado == 400 and datos['ok'] is False
    assert nodo.arbitro.salida(time.monotonic(), ROBOT) is None


def test_mando_llega_al_arbitro(arrancar):
    puerto, nodo = arrancar()
    estado, datos, r = post(puerto, '/mando', orden())
    assert (estado, datos) == (200, {'ok': True, 'motivo': 'ok'})
    assert r.getheader('Content-Type') == 'application/json'
    assert r.getheader('Access-Control-Allow-Origin') is None
    with nodo.lock:
        c = nodo.arbitro.salida(time.monotonic(), ROBOT)
        dueno = nodo.arbitro.estado(time.monotonic())['dueno']
    assert c is not None and c.lineal > 0
    assert dueno['ip'] == '127.0.0.1'


def test_los_rechazos_del_arbitro_salen_con_su_codigo(arrancar):
    puerto, nodo = arrancar()
    nodo.ctx = (True, 'emergencia', True)
    assert post(puerto, '/mando', orden())[0] == 409
    nodo.ctx = (True, 'web', False)
    estado, datos, _ = post(puerto, '/mando', orden(seq=1))
    assert (estado, datos['motivo']) == (503, 'sin mux')
    nodo.ctx = (False, 'web', True)
    estado, datos, _ = post(puerto, '/mando', orden(seq=2))
    assert (estado, datos['motivo']) == (503, 'sin enlace')


def test_emergencia_sin_clave_202(arrancar):
    puerto, nodo = arrancar()
    estado, datos, _ = post(puerto, '/emergencia', clave=None, tipo=None)
    assert estado == 202 and datos['ok'] is True
    with nodo.lock:
        c = nodo.arbitro.salida(time.monotonic(), ROBOT)
    assert c.emergencia


def test_emergencia_sin_content_length_tambien(arrancar):
    puerto, _ = arrancar()
    s = socket.create_connection(('127.0.0.1', puerto), timeout=2)
    try:
        s.sendall(b'POST /emergencia HTTP/1.0\r\n\r\n')
        assert s.recv(4096).startswith(b'HTTP/1.0 202')
    finally:
        s.close()


def test_rearmar_pide_clave(arrancar):
    puerto, nodo = arrancar()
    assert post(puerto, '/rearmar', clave=None, tipo=None)[0] == 401
    assert nodo.rearmes == 0
    assert post(puerto, '/rearmar', tipo=None)[0] == 202
    assert nodo.rearmes == 1


def test_ruta_post_desconocida_404(arrancar):
    puerto, _ = arrancar()
    assert post(puerto, '/otra', orden())[0] == 404


def test_las_rutas_get_siguen(arrancar):
    puerto, _ = arrancar()
    c = http.client.HTTPConnection('127.0.0.1', puerto, timeout=3)
    for ruta, esperado in (('/', 200), ('/3d', 200), ('/modelo', 503),
                           ('/nada', 404)):
        c.request('GET', ruta)
        r = c.getresponse()
        r.read()
        assert r.status == esperado, ruta
        c.close()


def test_el_servidor_contesta_por_ipv6_y_por_ipv4():
    """Sin IPv6, localhost tarda 300 ms en caer a IPv4 en el navegador."""
    from starcrawler_gui.servidor import crear_servidor, crear_manejador
    srv = crear_servidor(0, crear_manejador())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        puerto = srv.server_address[1]
        destinos = ['127.0.0.1']
        if srv.address_family == socket.AF_INET6:
            destinos.append('::1')
        for destino in destinos:
            with socket.create_connection((destino, puerto), timeout=2) as s:
                s.sendall(b'GET /modelo HTTP/1.0\r\n\r\n')
                assert s.recv(64).startswith(b'HTTP/1.0 ')
    finally:
        srv.shutdown()
        srv.server_close()


def test_la_emergencia_no_se_rechaza_por_el_content_length(arrancar):
    puerto, nodo = arrancar()
    with socket.create_connection(('127.0.0.1', puerto), timeout=2) as s:
        s.sendall(b'POST /emergencia HTTP/1.0\r\nContent-Length: abc\r\n\r\n')
        assert s.recv(64).startswith(b'HTTP/1.0 202')
    assert any('Emergencia web' in e for e in nodo.arbitro.sacar_eventos())


def test_un_cliente_lento_no_manda(arrancar):
    """La orden que tarda mas de 0,3 s en llegar ya la ha abortado la pagina."""
    puerto, _ = arrancar()
    cuerpo = json.dumps({'sesion': 'sesionA1', 'seq': 0, 'avance': 1.0,
                         'giro': 0.0, 'lento': False, 'delanteras': 0,
                         'traseras': 0, 'inclinar': None,
                         'preset': None}).encode()
    with socket.create_connection(('127.0.0.1', puerto), timeout=3) as s:
        time.sleep(0.4)
        s.sendall(b'POST /mando HTTP/1.0\r\nContent-Type: application/json\r\n'
                  + CABECERA_CLAVE.encode() + b': ' + CLAVE.encode()
                  + b'\r\nContent-Length: ' + str(len(cuerpo)).encode()
                  + b'\r\n\r\n' + cuerpo)
        respuesta = s.recv(1024)
    assert respuesta.startswith(b'HTTP/1.0 409') and b'atrasada' in respuesta


def test_en_doble_pila_la_ip_ipv4_sale_limpia():
    from starcrawler_gui.servidor import crear_servidor
    nodo = Nodo()
    srv = crear_servidor(0, crear_manejador(
        arbitro=nodo.arbitro, lock=nodo.lock, clave=CLAVE, mando=True,
        contexto=nodo.contexto, rearmar=nodo.rearmar))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        with socket.create_connection(('127.0.0.1', srv.server_address[1]),
                                      timeout=2) as s:
            s.sendall(b'POST /emergencia HTTP/1.0\r\n\r\n')
            s.recv(64)
        assert 'Emergencia web desde 127.0.0.1' in nodo.arbitro.sacar_eventos()
    finally:
        srv.shutdown()
        srv.server_close()
