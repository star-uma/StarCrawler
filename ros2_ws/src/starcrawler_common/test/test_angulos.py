"""Tests de la traduccion de angulos. Corren en el PC, sin ROS.

Esta conversion es de las pocas cosas del proyecto en las que un error de
signo no da un fallo evidente: el robot funciona, pero mueve los brazos
al reves. De ahi que se pruebe contra los valores concretos del TFG y no
solo contra si misma.
"""
import math

from starcrawler_common.angulos import (
    N_ORUGAS,
    NOMBRES,
    es_espejada,
    elevacion_a_encoder_deg,
    encoder_a_elevacion_rad,
)


def test_el_orden_es_fr_fl_rr_rl():
    assert NOMBRES == ('FR', 'FL', 'RR', 'RL')
    assert len(NOMBRES) == N_ORUGAS


def test_fl_y_rr_son_las_espejadas():
    assert [es_espejada(i) for i in range(N_ORUGAS)] == [False, True, True, False]


# --- Contra los valores del TFG -------------------------------------------

def test_horizontal_es_180_en_las_cuatro():
    assert [elevacion_a_encoder_deg(i, 0.0) for i in range(N_ORUGAS)] == [180.0] * 4


def test_vertical_arriba_es_90_270_270_90():
    """El TFG documenta esta pose como {90, 270, 270, 90}."""
    noventa = math.radians(90)
    obtenido = [round(elevacion_a_encoder_deg(i, noventa)) for i in range(N_ORUGAS)]
    assert obtenido == [90, 270, 270, 90]


def test_vertical_abajo_es_270_90_90_270():
    """La pose simetrica: -90 grados de elevacion."""
    menos90 = math.radians(-90)
    obtenido = [round(elevacion_a_encoder_deg(i, menos90)) for i in range(N_ORUGAS)]
    assert obtenido == [270, 90, 90, 270]


def test_en_elevacion_la_pose_es_simetrica():
    """Lo que gana el convenio de elevacion: subir es +90 en las cuatro,
    aunque en encoder cada par vaya a un lado."""
    for i in range(N_ORUGAS):
        encoder = elevacion_a_encoder_deg(i, math.radians(90))
        assert math.isclose(math.degrees(encoder_a_elevacion_rad(i, encoder)),
                            90.0, rel_tol=1e-9)


# --- Coherencia ------------------------------------------------------------

def test_la_conversion_va_y_vuelve():
    for i in range(N_ORUGAS):
        for rad in (-1.5, -0.7, -0.1, 0.0, 0.1, 0.7, 1.5):
            ida = elevacion_a_encoder_deg(i, rad)
            assert math.isclose(encoder_a_elevacion_rad(i, ida), rad, abs_tol=1e-12)


def test_la_conversion_va_y_vuelve_en_encoder():
    for i in range(N_ORUGAS):
        for deg in (85.0, 120.0, 180.0, 240.0, 275.0):
            ida = encoder_a_elevacion_rad(i, deg)
            assert math.isclose(elevacion_a_encoder_deg(i, ida), deg, abs_tol=1e-9)


def test_subir_el_brazo_aleja_del_horizontal_en_las_cuatro():
    """Elevacion positiva siempre se separa de 180, a un lado o a otro."""
    for i in range(N_ORUGAS):
        encoder = elevacion_a_encoder_deg(i, math.radians(30))
        assert abs(encoder - 180.0) > 1.0


def test_los_pares_espejados_van_en_sentidos_opuestos():
    """FR y FL, ante la misma elevacion, se separan de 180 al reves."""
    e = math.radians(45)
    fr = elevacion_a_encoder_deg(0, e) - 180.0
    fl = elevacion_a_encoder_deg(1, e) - 180.0
    assert fr < 0 and fl > 0
    assert math.isclose(abs(fr), abs(fl), rel_tol=1e-12)


def test_los_limites_software_en_elevacion():
    """85 y 275 en encoder son los topes; en elevacion salen simetricos."""
    for i in range(N_ORUGAS):
        a = math.degrees(encoder_a_elevacion_rad(i, 85.0))
        b = math.degrees(encoder_a_elevacion_rad(i, 275.0))
        assert math.isclose(abs(a), 95.0, rel_tol=1e-9)
        assert math.isclose(abs(b), 95.0, rel_tol=1e-9)
        assert a * b < 0        # uno positivo y otro negativo
