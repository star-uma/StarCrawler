"""Tests del mando web. Corren en el PC, sin ROS ni navegador."""
import json
import math

import pytest

from starcrawler_common.orugas import MAX_ANGULAR, MAX_LINEAL
from starcrawler_gui.mando_web import (
    CABECERA_CLAVE,
    Arbitro,
    OrdenWeb,
    comprobar_cabeceras,
    validar_mando,
)

CLAVE = 'a1b2c3d4e5'
SESION = 'sesionA1'
OTRA = 'sesionB2'
IP = '192.168.1.20'

BASE = {'sesion': SESION, 'seq': 0, 'avance': 0.0, 'giro': 0.0,
        'lento': False, 'delanteras': 0, 'traseras': 0,
        'inclinar': None, 'preset': None}


def cuerpo(**cambios):
    d = dict(BASE)
    d.update(cambios)
    return json.dumps(d).encode('utf-8')


def orden(seq, sesion=SESION, **cambios):
    d = dict(BASE)
    d.update(cambios, sesion=sesion, seq=seq)
    return OrdenWeb(**d)


def robot(elev_deg=(0, 0, 0, 0), encoder_ok=True, enlace=True):
    return {'elev': [math.radians(a) for a in elev_deg],
            'encoder_ok': [encoder_ok] * 4, 'enlace': enlace}


def mandar(a, o, t, enlace=True, activa='web', mux_ok=True, ip=IP):
    return a.mando(o, ip, t, enlace, activa, mux_ok)


# ─── Cuerpo ──────────────────────────────────────────────────────────────────

def test_cuerpo_valido():
    o = validar_mando(cuerpo(seq=7, avance=1, giro=-0.5, lento=True,
                             delanteras=-1, inclinar='izq', preset=3))
    assert o.seq == 7 and o.avance == 1.0 and type(o.avance) is float
    assert o.giro == -0.5 and o.lento and o.delanteras == -1
    assert o.inclinar == 'izq' and o.preset == 3


@pytest.mark.parametrize('texto', [
    b'{"avance": NaN}', b'{"avance": Infinity}', b'{"avance": -Infinity}'])
def test_constantes_no_json_se_rechazan(texto):
    crudo = cuerpo().decode().replace('"avance": 0.0',
                                      texto.decode()[1:-1])
    with pytest.raises(ValueError):
        validar_mando(crudo.encode())


def test_1e309_es_infinito_y_se_rechaza():
    crudo = cuerpo().decode().replace('"giro": 0.0', '"giro": 1e309')
    with pytest.raises(ValueError):
        validar_mando(crudo.encode())


def test_entero_enorme_no_revienta():
    crudo = cuerpo().decode().replace('"giro": 0.0', '"giro": 1' + '0' * 400)
    with pytest.raises(ValueError):
        validar_mando(crudo.encode())


@pytest.mark.parametrize('cambios', [
    {'avance': None}, {'giro': 'false'}, {'avance': True},
    {'avance': 1.01}, {'giro': -1.5},
    {'delanteras': True}, {'delanteras': 1.0}, {'delanteras': 2},
    {'delanteras': None}, {'traseras': '1'},
    {'lento': 0}, {'lento': None}, {'lento': 'false'},
    {'seq': -1}, {'seq': 1.0}, {'seq': True},
    {'inclinar': 'adelante'}, {'inclinar': 1}, {'inclinar': ''},
    {'preset': 4}, {'preset': -1}, {'preset': 1.0}, {'preset': True},
    {'sesion': 'corta'}, {'sesion': 'x' * 33}, {'sesion': 'con espacio'},
    {'sesion': 'acentuadá1'}, {'sesion': 12345678},
])
def test_valores_invalidos(cambios):
    with pytest.raises(ValueError):
        validar_mando(cuerpo(**cambios))


def test_claves_de_mas_o_de_menos():
    with pytest.raises(ValueError):
        validar_mando(cuerpo(extra=1))
    d = dict(BASE)
    del d['preset']
    with pytest.raises(ValueError):
        validar_mando(json.dumps(d).encode())


def test_clave_repetida():
    crudo = cuerpo().decode()[:-1] + ', "seq": 5}'
    with pytest.raises(ValueError):
        validar_mando(crudo.encode())


@pytest.mark.parametrize('crudo', [
    b'', b'no es json', b'[1, 2]', b'null', b'\xff\xfe{', b'[' * 1000,
    b'{"a": 1}' + b' ' * 1024])
def test_basura(crudo):
    with pytest.raises(ValueError):
        validar_mando(crudo)


# ─── Cabeceras, sin leer el cuerpo ───────────────────────────────────────────

def cabeceras(clave=CLAVE, tipo='application/json', longitud='120'):
    c = {}
    if clave is not None:
        c[CABECERA_CLAVE] = clave
    if tipo is not None:
        c['Content-Type'] = tipo
    if longitud is not None:
        c['Content-Length'] = longitud
    return c


def codigo(ruta, c, mando=True, clave=CLAVE):
    r = comprobar_cabeceras(ruta, c, mando, clave)
    return None if r is None else r[0]


def test_cabeceras_buenas():
    assert codigo('/mando', cabeceras()) is None
    assert codigo('/mando', cabeceras(tipo='application/json; charset=utf-8')) is None
    assert codigo('/rearmar', cabeceras(tipo=None, longitud='0')) is None


def test_sin_mando_todo_es_403():
    for ruta in ('/mando', '/emergencia', '/rearmar'):
        assert codigo(ruta, cabeceras(), mando=False) == 403


def test_clave():
    assert codigo('/mando', cabeceras(clave=None)) == 401
    assert codigo('/mando', cabeceras(clave='otra')) == 401
    assert codigo('/rearmar', cabeceras(clave=None)) == 401
    # sin clave configurada no entra nadie, ni con la cabecera vacia
    assert codigo('/mando', cabeceras(clave=''), clave='') == 401


def test_la_cabecera_no_distingue_mayusculas():
    c = {'x-starcrawler-token': CLAVE, 'content-type': 'Application/JSON',
         'content-length': '10'}
    assert codigo('/mando', c) is None


def test_emergencia_no_pide_clave_ni_tipo():
    assert codigo('/emergencia', cabeceras(clave=None, tipo=None,
                                           longitud='0')) is None
    assert codigo('/emergencia', cabeceras(clave=None, tipo=None,
                                           longitud=None)) is None


def test_tipo():
    assert codigo('/mando', cabeceras(tipo='text/plain')) == 415
    assert codigo('/mando', cabeceras(tipo=None)) == 415


def test_longitud():
    assert codigo('/mando', cabeceras(longitud=None)) == 411
    assert codigo('/mando', cabeceras(longitud='0')) == 411
    assert codigo('/mando', cabeceras(longitud='1025')) == 413
    assert codigo('/mando', cabeceras(longitud='-5')) == 400
    assert codigo('/mando', cabeceras(longitud='12a')) == 400
    assert codigo('/rearmar', cabeceras(longitud=None)) == 411
    assert codigo('/emergencia', cabeceras(longitud='5000')) == 413


def test_el_orden_de_los_rechazos():
    """Sin mando ni se mira la clave; sin clave ni se mira el tipo."""
    malas = cabeceras(clave=None, tipo='text/plain', longitud='9999')
    assert codigo('/mando', malas, mando=False) == 403
    assert codigo('/mando', malas) == 401
    assert codigo('/mando', dict(malas, **{CABECERA_CLAVE: CLAVE})) == 415


def test_ruta_desconocida():
    assert codigo('/otra', cabeceras()) == 404


# ─── Dueno, seq y frescura ───────────────────────────────────────────────────

def test_el_primero_manda_y_va_al_log_con_la_ip():
    a = Arbitro()
    assert mandar(a, orden(0), 0.0) == (200, 'ok')
    eventos = a.sacar_eventos()
    assert len(eventos) == 1 and IP in eventos[0] and SESION[:6] in eventos[0]
    assert a.estado(0.0)['dueno'] == {'id': SESION[:6], 'ip': IP}


def test_otro_puesto_espera_a_que_el_dueno_calle_1_s():
    a = Arbitro()
    mandar(a, orden(0), 0.0)
    assert mandar(a, orden(0, OTRA), 0.5, ip='10.0.0.9') == (409, 'otro puesto')
    assert mandar(a, orden(0, OTRA), 0.99, ip='10.0.0.9')[0] == 409
    a.sacar_eventos()
    assert mandar(a, orden(0, OTRA), 1.01, ip='10.0.0.9')[0] == 200
    assert '10.0.0.9' in a.sacar_eventos()[0]
    # y ahora el primero es el que espera
    assert mandar(a, orden(1), 1.2) == (409, 'otro puesto')


def test_el_nuevo_dueno_empieza_seq_de_cero():
    a = Arbitro()
    mandar(a, orden(50), 0.0)
    assert mandar(a, orden(0, OTRA), 2.0)[0] == 200


def test_seq_tiene_que_crecer():
    a = Arbitro()
    assert mandar(a, orden(5), 0.0)[0] == 200
    assert mandar(a, orden(5), 0.05) == (409, 'seq no crece')
    assert mandar(a, orden(4), 0.06)[0] == 409
    assert mandar(a, orden(6), 0.07)[0] == 200


def test_la_orden_caduca_a_los_02_s():
    a = Arbitro()
    mandar(a, orden(0, avance=1.0), 0.0)
    assert a.salida(0.19, robot()).lineal == pytest.approx(MAX_LINEAL)
    assert a.salida(0.2, robot()) is None
    assert a.estado(0.2)['pedido'] == [0.0, 0.0]


def test_sin_ordenes_no_se_publica():
    assert Arbitro().salida(0.0, robot()) is None


def test_el_dueno_callado_no_se_muestra():
    a = Arbitro()
    mandar(a, orden(0), 0.0)
    assert a.estado(1.5)['dueno'] is None


def test_sin_mux():
    a = Arbitro()
    assert mandar(a, orden(0), 0.0, mux_ok=False) == (503, 'sin mux')
    assert a.salida(0.01, robot()) is None


# ─── Emergencia y enlace ─────────────────────────────────────────────────────

def test_emergencia_para_y_suelta_al_dueno():
    a = Arbitro()
    mandar(a, orden(0, avance=1.0, delanteras=1), 0.0)
    a.emergencia(0.05, IP)
    c = a.salida(0.1, robot())
    assert c.emergencia and c.lineal == 0.0 and c.angular == 0.0
    assert c.incremento == [0, 0, 0, 0] and not c.usar_posicion
    assert a.estado(0.1)['dueno'] is None
    # pasada la ventana no queda orden que publicar
    assert a.salida(0.26, robot()) is None


def test_emergencia_repetida_alarga_la_ventana_y_se_anota_una_vez():
    a = Arbitro()
    for i in range(5):
        a.emergencia(0.1 * i, IP)
    assert len(a.sacar_eventos()) == 1
    assert a.salida(0.55, robot()).emergencia
    assert a.salida(0.61, robot()) is None


def test_con_la_ventana_abierta_no_se_acepta_mando():
    a = Arbitro()
    a.emergencia(0.0)
    assert mandar(a, orden(0), 0.1) == (409, 'emergencia')
    assert mandar(a, orden(1), 0.3)[0] == 200


def test_con_el_enganche_puesto_409():
    a = Arbitro()
    assert mandar(a, orden(0), 0.0, activa='emergencia') == (409, 'emergencia')


def test_sin_enlace_solo_pasa_lo_neutro():
    a = Arbitro()
    assert mandar(a, orden(0, avance=0.5), 0.0, enlace=False) == \
        (503, 'sin enlace')
    assert mandar(a, orden(1, preset=1), 0.01, enlace=False)[0] == 503
    assert mandar(a, orden(2), 0.02, enlace=False) == (200, 'ok')


def test_sin_enlace_suelta_la_orden_anterior():
    a = Arbitro()
    mandar(a, orden(0, avance=1.0), 0.0)
    mandar(a, orden(1, avance=1.0), 0.05, enlace=False)
    assert a.salida(0.06, robot()) is None


# ─── Signos ──────────────────────────────────────────────────────────────────

def salida_de(**cambios):
    a = Arbitro()
    mandar(a, orden(0, **cambios), 0.0)
    return a.salida(0.0, robot())


def test_giro_a_la_derecha_es_angular_negativo():
    c = salida_de(giro=1.0)
    assert c.lineal == 0.0 and c.angular == pytest.approx(-MAX_ANGULAR)


def test_avance():
    c = salida_de(avance=-1.0)
    assert c.lineal == pytest.approx(-MAX_LINEAL)


def test_delanteras_suben():
    assert salida_de(delanteras=1).incremento == [1, 1, 0, 0]
    assert salida_de(traseras=-1).incremento == [0, 0, -1, -1]


def test_inclinar_adelante():
    assert salida_de(inclinar='arriba').incremento == [1, 1, -1, -1]


def test_lento_es_la_mitad():
    c = salida_de(avance=1.0, giro=1.0, lento=True)
    assert c.lineal == pytest.approx(MAX_LINEAL / 2)
    assert c.angular == pytest.approx(-MAX_ANGULAR / 2)


def test_los_topes_son_los_del_arbitro():
    a = Arbitro(max_lineal=0.1, max_angular=1.0, factor_lento=0.25)
    mandar(a, orden(0, avance=1.0, giro=-1.0, lento=True), 0.0)
    c = a.salida(0.0, robot())
    assert c.lineal == pytest.approx(0.025) and c.angular == pytest.approx(0.25)
    assert a.estado(0.0)['pedido'] == [pytest.approx(0.025),
                                       pytest.approx(0.25)]


# ─── Presets ─────────────────────────────────────────────────────────────────

class Puesto:
    """Una pagina que manda a 20 Hz y un temporizador a 50 Hz."""

    def __init__(self, arbitro=None):
        self.a = arbitro or Arbitro()
        self.seq = 0
        self.t = 0.0

    def durante(self, segundos, rob=None, **cambios):
        rob = rob or robot()
        fin = self.t + segundos
        c = None
        while self.t < fin - 1e-9:
            if round(self.t * 1000) % 50 == 0:
                assert mandar(self.a, orden(self.seq, **cambios), self.t)[0] == 200
                self.seq += 1
            c = self.a.salida(self.t, rob)
            self.t = round(self.t + 0.01, 3)
        return c

    def preset(self):
        return self.a.estado(self.t)['preset']


def test_preset_esperando_hasta_s_preset():
    p = Puesto()
    c = p.durante(0.45, preset=3)
    assert not c.usar_posicion
    assert p.preset() == {'k': 3, 'objetivo_deg': 90.0, 'estado': 'esperando'}


def test_preset_en_curso():
    p = Puesto()
    c = p.durante(0.6, preset=3)
    assert c.usar_posicion and c.incremento == [0, 0, 0, 0]
    assert c.objetivo_rad == [pytest.approx(math.pi / 2)] * 4
    assert p.preset()['estado'] == 'en curso'


def test_conducir_no_cancela_el_preset():
    p = Puesto()
    c = p.durante(0.6, preset=3, avance=1.0)
    assert c.usar_posicion and c.lineal == pytest.approx(MAX_LINEAL)


def test_ciclo_completo_del_preset():
    p = Puesto()
    p.durante(0.6, preset=2)
    assert p.preset()['estado'] == 'en curso'
    c = p.durante(0.1, rob=robot((44.0, 45.5, 46.9, 43.1)), preset=2)
    assert not c.usar_posicion
    assert p.preset()['estado'] == 'llegado'
    assert any('llegado' in e for e in p.a.sacar_eventos())
    # terminado: se ignora aunque se siga mandando el mismo valor
    c = p.durante(1.0, preset=2)
    assert not c.usar_posicion and p.preset()['estado'] == 'llegado'
    # otro valor lo rearma
    p.durante(0.05)
    assert p.preset()['estado'] == 'llegado'
    p.durante(0.6, preset=2)
    assert p.preset()['estado'] == 'en curso'


def test_soltar_antes_de_tiempo_cancela():
    p = Puesto()
    p.durante(0.3, preset=1)
    c = p.durante(0.5)
    assert not c.usar_posicion and p.preset()['estado'] == 'cancelado'


def test_orden_manual_cancela():
    p = Puesto()
    p.durante(0.6, preset=3)
    c = p.durante(0.05, preset=3, traseras=1)
    assert p.preset()['estado'] == 'cancelado'
    assert not c.usar_posicion and c.incremento == [0, 0, 1, 1]
    c = p.durante(1.0, preset=3)
    assert not c.usar_posicion


def test_preset_sin_encoder():
    p = Puesto()
    c = p.durante(0.6, rob=robot(encoder_ok=False), preset=3)
    assert not c.usar_posicion and p.preset()['estado'] == 'sin encoder'


def test_preset_sin_enlace():
    p = Puesto()
    p.durante(0.6, preset=3)
    c = p.a.salida(p.t, robot(enlace=False))
    assert not c.usar_posicion and p.preset()['estado'] == 'sin enlace'


def test_preset_tiempo():
    p = Puesto(Arbitro(preset_max_s=1.0))
    p.durante(1.4, preset=3)
    assert p.preset()['estado'] == 'en curso'
    c = p.durante(0.2, preset=3)
    assert not c.usar_posicion and p.preset()['estado'] == 'tiempo'


def test_sin_ordenes_se_olvida_el_preset():
    p = Puesto()
    p.durante(0.6, preset=3)
    p.t += 0.3
    assert p.a.salida(p.t, robot()) is None
    assert p.preset() is None
    # volver a mandarlo empieza de cero
    c = p.durante(0.2, preset=3)
    assert not c.usar_posicion and p.preset()['estado'] == 'esperando'


def test_la_emergencia_borra_el_preset():
    p = Puesto()
    p.durante(0.6, preset=3)
    p.a.emergencia(p.t)
    assert p.preset() is None


def test_estado_para_events():
    a = Arbitro(s_preset=0.7)
    e = a.estado(0.0, 'joy', True)
    assert e == {'habilitado': True, 's_preset': 0.7, 'activa': 'joy',
                 'dueno': None, 'preset': None, 'pedido': [0.0, 0.0],
                 'mux_ok': True}
    json.dumps(e)


# ─── Ordenes retrasadas en la red ────────────────────────────────────────────

def test_orden_atrasada_se_rechaza():
    a = Arbitro()
    assert a.mando(orden(0), IP, 5.0, True, 'web', True, t_conexion=4.9)[0] == 200
    assert a.mando(orden(1, avance=1.0), IP, 5.5, True, 'web', True,
                   t_conexion=5.1) == (409, 'orden atrasada')


def test_la_orden_de_antes_de_la_emergencia_no_revive_tras_rearmar():
    a = Arbitro()
    for s in range(3):
        t = 1.0 + s * 0.05
        assert a.mando(orden(s, avance=1.0), IP, t, True, 'web', True,
                       t_conexion=t)[0] == 200
    a.emergencia(2.0)
    # ya rearmado (activa=joy) llega la que se quedo en la red
    assert a.mando(orden(3, avance=1.0), IP, 2.25, True, 'joy', True,
                   t_conexion=1.99) == (409, 'orden anterior a la emergencia')
    assert a.mando(orden(4), IP, 2.3, True, 'joy', True, t_conexion=2.28)[0] == 200


def test_el_preset_necesita_una_orden_despues_de_s_preset():
    """La gracia de frescura de la ultima orden no cuenta como mantenido."""
    p = Puesto()
    p.durante(0.35, preset=3)
    c = None
    while p.t < 0.6:
        c = p.a.salida(p.t, robot()) or c
        p.t = round(p.t + 0.01, 3)
    assert c is None or not c.usar_posicion


def test_con_ordenes_cada_0_25_s_el_preset_no_arranca():
    """Por eso la pagina aborta /mando a los 180 ms, antes de caducar aqui."""
    a = Arbitro()
    t, seq, c = 0.0, 0, None
    while t < 2.0:
        if round(t * 100) % 25 == 0:
            assert mandar(a, orden(seq, preset=3), t)[0] == 200
            seq += 1
        c = a.salida(t, robot()) or c
        t = round(t + 0.01, 3)
    assert c is None or not c.usar_posicion
