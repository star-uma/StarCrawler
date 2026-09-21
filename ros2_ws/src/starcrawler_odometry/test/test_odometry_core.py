"""Tests de odometry_core. No necesitan ROS: corren en el PC con pytest."""
import math

from starcrawler_odometry.odometry_core import (
    velocidades_del_robot,
    integrar,
    normalizar_angulo,
    cuaternion_de_yaw,
)

RADIO = 0.0764
SEPARACION = 0.524


def test_ambas_bandas_igual_es_avance_recto():
    v, w = velocidades_del_robot(2.0, 2.0, RADIO, SEPARACION)
    assert math.isclose(v, 2.0 * RADIO, rel_tol=1e-9)
    assert math.isclose(w, 0.0, abs_tol=1e-12)


def test_bandas_opuestas_giran_sobre_el_sitio():
    v, w = velocidades_del_robot(-1.0, 1.0, RADIO, SEPARACION)
    assert math.isclose(v, 0.0, abs_tol=1e-12)
    assert math.isclose(w, 2.0 * RADIO / SEPARACION, rel_tol=1e-9)


def test_parado_es_parado():
    v, w = velocidades_del_robot(0.0, 0.0, RADIO, SEPARACION)
    assert v == 0.0 and w == 0.0


def test_recta_avanza_en_x():
    x, y, th = integrar(0.0, 0.0, 0.0, 1.0, 0.0, 1.0)
    assert math.isclose(x, 1.0, rel_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(th, 0.0, abs_tol=1e-12)


def test_recta_orientada_45_grados():
    th0 = math.pi / 4
    x, y, th = integrar(0.0, 0.0, th0, math.sqrt(2.0), 0.0, 1.0)
    assert math.isclose(x, 1.0, rel_tol=1e-9)
    assert math.isclose(y, 1.0, rel_tol=1e-9)
    assert math.isclose(th, th0, rel_tol=1e-9)


def test_giro_sobre_el_sitio_no_desplaza():
    x, y, th = integrar(0.0, 0.0, 0.0, 0.0, math.pi / 2, 1.0)
    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(th, math.pi / 2, rel_tol=1e-9)


def test_cuarto_de_circunferencia():
    """Un cuarto de vuelta a la izquierda con radio 1 acaba en (1, 1).

    El radio es v/w, asi que para radio 1 hacen falta v y w iguales."""
    x, y, th = integrar(0.0, 0.0, 0.0, math.pi / 2, math.pi / 2, 1.0)
    assert math.isclose(x, 1.0, rel_tol=1e-6)
    assert math.isclose(y, 1.0, rel_tol=1e-6)
    assert math.isclose(th, math.pi / 2, rel_tol=1e-9)


def test_dt_cero_no_cambia_nada():
    assert integrar(1.0, 2.0, 0.5, 9.0, 9.0, 0.0) == (1.0, 2.0, 0.5)


def test_dt_negativo_no_cambia_nada():
    assert integrar(1.0, 2.0, 0.5, 9.0, 9.0, -0.1) == (1.0, 2.0, 0.5)


def test_el_angulo_se_normaliza():
    _, _, th = integrar(0.0, 0.0, 3.0, 0.0, 1.0, 1.0)
    assert -math.pi <= th <= math.pi


def test_normalizar_angulo():
    assert math.isclose(normalizar_angulo(0.0), 0.0, abs_tol=1e-12)
    # +pi y -pi son el mismo angulo: se compara la direccion, no el signo
    for entrada in (3 * math.pi, -3 * math.pi):
        th = normalizar_angulo(entrada)
        assert math.isclose(abs(th), math.pi, rel_tol=1e-9)
        assert math.isclose(math.cos(th), -1.0, rel_tol=1e-9)


def test_cuaternion_identidad():
    x, y, z, w = cuaternion_de_yaw(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert math.isclose(w, 1.0, rel_tol=1e-9)


def test_cuaternion_media_vuelta():
    x, y, z, w = cuaternion_de_yaw(math.pi)
    assert math.isclose(z, 1.0, rel_tol=1e-9)
    assert math.isclose(w, 0.0, abs_tol=1e-9)


def test_cuaternion_siempre_unitario():
    for th in (-2.0, -0.3, 0.0, 0.7, 2.9):
        x, y, z, w = cuaternion_de_yaw(th)
        assert math.isclose(x * x + y * y + z * z + w * w, 1.0, rel_tol=1e-9)


def test_recorrido_completo_vuelve_al_origen():
    """Cuatro cuartos de vuelta cierran el cuadrado: la integracion no
    puede acumular deriva en un caso tan simple."""
    x = y = th = 0.0
    for _ in range(4):
        x, y, th = integrar(x, y, th, 1.0, math.pi / 2, 1.0)
    assert math.isclose(x, 0.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)
