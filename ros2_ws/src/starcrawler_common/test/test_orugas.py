"""Tests del esquema de mando compartido. Corren en el PC, sin ROS."""
import pytest

from starcrawler_common.orugas import (
    MAX_ANGULAR,
    MAX_LINEAL,
    PRESETS_DEG,
    componer_incremento,
    inclinacion,
    traccion,
)


def test_avanzar_da_lineal_positivo():
    assert traccion(1.0, 0.0) == (pytest.approx(MAX_LINEAL), 0.0)


def test_girar_a_la_derecha_da_angular_negativo():
    lineal, angular = traccion(0.0, 1.0)
    assert lineal == 0.0 and angular == pytest.approx(-MAX_ANGULAR)


def test_el_factor_lento_escala_los_dos():
    assert traccion(1.0, 1.0, 0.5) == (pytest.approx(MAX_LINEAL / 2),
                                       pytest.approx(-MAX_ANGULAR / 2))


def test_pares():
    assert componer_incremento(1, 0) == [1, 1, 0, 0]
    assert componer_incremento(0, -1) == [0, 0, -1, -1]
    assert componer_incremento(1, -1) == [1, 1, -1, -1]


def test_inclinaciones():
    assert inclinacion('arriba') == [1, 1, -1, -1]
    assert inclinacion('abajo') == [-1, -1, 1, 1]
    assert inclinacion('izq') == [-1, 1, -1, 1]
    assert inclinacion('der') == [1, -1, 1, -1]
    assert inclinacion(None) == [0, 0, 0, 0]


def test_la_suma_satura_por_oruga():
    # delanteras suben y ademas se inclina adelante: siguen en +1
    assert componer_incremento(1, 0, inclinacion('arriba')) == [1, 1, -1, -1]
    # delanteras bajan contra la inclinacion adelante: se anulan
    assert componer_incremento(-1, 0, inclinacion('arriba')) == [0, 0, -1, -1]


def test_presets_del_ds4():
    assert PRESETS_DEG == (-45.0, 0.0, 45.0, 90.0)
