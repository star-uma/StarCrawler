"""Nivelado: que esquina baja cada desnivel y como se mueven los brazos."""

import math

import pytest

from starcrawler_teleop.nivelado_core import (AjustesNivelado, Nivelador,
                                              cabeceo_balanceo, desniveles)

FR, FL, RR, RL = range(4)
GRADO = math.radians(1.0)


def correr(nivelador, cabeceo, balanceo, q, segundos, dt=0.02):
    for _ in range(int(round(segundos / dt))):
        q = nivelador.paso(cabeceo, balanceo, q, dt)
    return q


def test_cuaternion_de_cabeceo_y_balanceo():
    c, b = math.radians(10.0), math.radians(-5.0)
    # cuaternion ZYX con yaw 0
    qw = math.cos(b / 2) * math.cos(c / 2)
    qx = math.sin(b / 2) * math.cos(c / 2)
    qy = math.cos(b / 2) * math.sin(c / 2)
    qz = -math.sin(b / 2) * math.sin(c / 2)
    assert cabeceo_balanceo(qx, qy, qz, qw) == pytest.approx((c, b))


def test_morro_abajo_bajan_las_delanteras():
    e = desniveles(math.radians(10.0), 0.0)
    assert e[FR] < 0 and e[FL] < 0 and e[RR] > 0 and e[RL] > 0


def test_lado_izquierdo_arriba_bajan_las_derechas():
    e = desniveles(0.0, math.radians(10.0))
    assert e[FR] < 0 and e[RR] < 0 and e[FL] > 0 and e[RL] > 0


def test_en_llano_no_se_mueve_nada():
    n = Nivelador()
    q = correr(n, 0.5 * GRADO, -0.5 * GRADO, [0.1, 0.1, 0.0, 0.0], 2.0)
    assert q == pytest.approx([0.1, 0.1, 0.0, 0.0])


def test_morro_abajo_baja_las_delanteras_y_no_levanta_las_traseras():
    n = Nivelador()
    q = correr(n, 10 * GRADO, 0.0, [0.0] * 4, 1.0)
    assert q[FR] < 0 and q[FL] < 0
    assert q[RR] == 0.0 and q[RL] == 0.0


def test_va_a_la_velocidad_del_firmware():
    n = Nivelador()
    q = correr(n, 30 * GRADO, 0.0, [0.0] * 4, 1.0)
    assert q[FR] == pytest.approx(-AjustesNivelado().vel_max * 1.0, rel=0.05)


def test_recoge_el_brazo_que_empuja_en_la_esquina_alta_hasta_horizontal():
    n = Nivelador()
    q = correr(n, -10 * GRADO, 0.0, [-0.05, -0.05, 0.0, 0.0], 3.0)
    assert q[FR] == 0.0 and q[FL] == 0.0       # delanteras: estaban empujando
    assert q[RR] < 0 and q[RL] < 0             # traseras: bajan


def test_no_baja_mas_del_tope():
    a = AjustesNivelado()
    n = Nivelador(a)
    q = [0.0] * 4
    for _ in range(3000):
        q = n.paso(30 * GRADO, 0.0, q, 0.02)
    assert min(q) == pytest.approx(a.bajada_max)


def test_el_objetivo_no_se_aleja_del_brazo_que_no_llega():
    a = AjustesNivelado()
    n = Nivelador(a)
    q = correr(n, 30 * GRADO, 0.0, [0.0] * 4, 0.02)
    for _ in range(500):                       # el brazo se queda en 0
        q = n.paso(30 * GRADO, 0.0, [0.0] * 4, 0.02)
    assert q[FR] == pytest.approx(-a.adelanto_max)


def test_un_brazo_levantado_no_se_levanta_mas():
    n = Nivelador()
    q = correr(n, 10 * GRADO, 0.0, [0.3, 0.3, 0.3, 0.3], 1.0)
    assert q[RR] == pytest.approx(0.3) and q[RL] == pytest.approx(0.3)
    assert q[FR] < 0.3


def test_encoder_caido_da_objetivo_finito_y_no_rompe_los_demas():
    n = Nivelador()
    medidas = [float('nan'), 0.0, 0.0, 0.0]
    for _ in range(50):
        q = n.paso(10 * GRADO, 0.0, medidas, 0.02)
        # El firmware descarta la orden entera si un objetivo no es finito
        assert all(math.isfinite(v) for v in q)
    assert q[FR] == 0.0 and q[FL] < 0


def test_encoder_caido_se_queda_en_su_ultima_medida():
    n = Nivelador()
    n.paso(10 * GRADO, 0.0, [-0.2, 0.0, 0.0, 0.0], 0.02)
    q = n.paso(10 * GRADO, 0.0, [float('nan'), 0.0, 0.0, 0.0], 0.02)
    assert q[FR] == pytest.approx(-0.2)


def test_vuelve_el_encoder_y_se_parte_de_la_medida():
    n = Nivelador()
    for _ in range(25):
        n.paso(10 * GRADO, 0.0, [float('nan'), 0.0, 0.0, 0.0], 0.02)
    q = n.paso(10 * GRADO, 0.0, [0.2, 0.0, 0.0, 0.0], 0.02)
    assert 0.0 < q[FR] < 0.2


def test_brazo_por_debajo_de_la_cota_ni_sube_ni_baja():
    q = Nivelador().paso(20 * GRADO, 0.0, [math.radians(-75), 0.0, 0.0, 0.0], 0.02)
    assert q[FR] == pytest.approx(math.radians(-75))


def test_primero_recoge_y_tras_la_cuesta_vuelve_a_horizontal():
    n = Nivelador()
    # Subiendo (morro arriba) bajan las traseras
    q = correr(n, -10 * GRADO, 0.0, [0.0] * 4, 5.0)
    assert q[RR] < -0.2 and q[RL] < -0.2
    # Arriba, el chasis queda morro abajo: primero recoge las traseras
    q = n.paso(5 * GRADO, 0.0, q, 0.02)
    assert q[FR] == 0.0 and q[FL] == 0.0
    q = correr(n, 5 * GRADO, 0.0, q, 10.0)
    assert q[RR] == 0.0 and q[RL] == 0.0
