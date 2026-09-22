"""Tests de sim_core. No necesitan ROS: corren en el PC con pytest."""
import math

from starcrawler_sim.sim_core import (
    N_ORUGAS,
    RobotSimulado,
    VEL_ELEVACION_DPS,
    rate_limit,
    es_espejada,
    elevacion_a_encoder_deg,
    encoder_a_elevacion_rad,
    ERR_WATCHDOG,
    ERR_CAN,
    ERR_ENCODER,
)

DT = 0.02


def robot_activo(**kwargs):
    """Robot con una consigna fresca, para que no salte el watchdog."""
    r = RobotSimulado(**kwargs)
    r.consigna_traccion(0.0, 0.0)
    r.consigna_orugas([0] * N_ORUGAS, [180.0] * N_ORUGAS, False, False)
    return r


# --- rate_limit -----------------------------------------------------------

def test_rate_limit_no_se_pasa_del_objetivo():
    assert rate_limit(0.0, 10.0, 100.0) == 10.0


def test_rate_limit_sube_a_saltos():
    assert rate_limit(0.0, 100.0, 4.0) == 4.0


def test_rate_limit_tambien_baja():
    assert rate_limit(0.0, -100.0, 4.0) == -4.0


# --- Arranque y watchdog ---------------------------------------------------

def test_arranca_en_estado_seguro():
    r = RobotSimulado()
    assert r.seguridad is True
    assert r.angulo == [180.0] * N_ORUGAS


def test_sin_consignas_salta_el_watchdog():
    r = robot_activo()
    for _ in range(100):          # 2 s sin nadie mandando
        r.avanzar(DT)
    assert r.seguridad is True
    assert r.vel_izq_dps == 0.0 and r.vel_der_dps == 0.0


def test_con_consignas_frescas_no_salta():
    r = robot_activo()
    for _ in range(100):
        r.consigna_traccion(10.0, 10.0)
        r.avanzar(DT)
    assert r.seguridad is False


def test_el_watchdog_pone_su_bit():
    r = RobotSimulado()
    r.avanzar(DT)
    assert r.bits_error & ERR_WATCHDOG


# --- Traccion --------------------------------------------------------------

def test_la_traccion_no_salta_de_golpe():
    r = robot_activo()
    r.consigna_traccion(400.0, 400.0)
    r.avanzar(DT)
    # 400 dps/s durante 20 ms = 8 dps como mucho
    assert 0 < r.vel_izq_dps <= 8.0 + 1e-9


def test_la_traccion_acaba_llegando():
    r = robot_activo()
    for _ in range(100):
        r.consigna_traccion(40.0, -40.0)
        r.avanzar(DT)
    assert math.isclose(r.vel_izq_dps, 40.0, rel_tol=1e-6)
    assert math.isclose(r.vel_der_dps, -40.0, rel_tol=1e-6)


def test_la_emergencia_para_la_traccion():
    r = robot_activo()
    for _ in range(50):
        r.consigna_traccion(40.0, 40.0)
        r.avanzar(DT)
    assert r.vel_izq_dps > 0
    r.consigna_orugas([0] * N_ORUGAS, [180.0] * N_ORUGAS, False, True)
    r.avanzar(DT)
    assert r.vel_izq_dps == 0.0 and r.vel_der_dps == 0.0


# --- Elevacion -------------------------------------------------------------

def test_el_incremento_mueve_a_la_velocidad_del_stepper():
    r = robot_activo()
    r.consigna_orugas([1, 0, 0, 0], [180.0] * N_ORUGAS, False, False)
    for _ in range(50):           # 1 s
        r.consigna_orugas([1, 0, 0, 0], [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert math.isclose(r.angulo[0], 180.0 + VEL_ELEVACION_DPS, rel_tol=1e-3)


def test_solo_se_mueve_la_oruga_comandada():
    r = robot_activo()
    for _ in range(50):
        r.consigna_orugas([1, 0, 0, 0], [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert r.angulo[1] == 180.0 and r.angulo[2] == 180.0 and r.angulo[3] == 180.0


def test_el_incremento_negativo_baja():
    r = robot_activo()
    for _ in range(50):
        r.consigna_orugas([-1, 0, 0, 0], [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert r.angulo[0] < 180.0


def test_no_se_pasa_del_limite_superior():
    r = robot_activo()
    for _ in range(2000):         # 40 s, de sobra para llegar al tope
        r.consigna_orugas([1] * N_ORUGAS, [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert all(a == 275.0 for a in r.angulo)


def test_no_se_pasa_del_limite_inferior():
    r = robot_activo()
    for _ in range(2000):
        r.consigna_orugas([-1] * N_ORUGAS, [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert all(a == 85.0 for a in r.angulo)


# --- Control de posicion ---------------------------------------------------

def test_la_posicion_llega_al_objetivo_y_para():
    r = robot_activo()
    objetivo = [200.0] * N_ORUGAS
    for _ in range(400):
        r.consigna_orugas([0] * N_ORUGAS, objetivo, True, False)
        r.avanzar(DT)
    assert all(abs(a - 200.0) <= 0.5 for a in r.angulo)


def test_la_posicion_no_se_pasa_de_largo():
    """Con dt grande, el paso no puede saltarse el objetivo."""
    r = robot_activo()
    r.consigna_orugas([0] * N_ORUGAS, [181.0] * N_ORUGAS, True, False)
    r.avanzar(10.0)               # un paso enorme a proposito
    assert all(a <= 181.0 + 1e-9 for a in r.angulo)


def test_sin_encoders_no_hay_lazo_cerrado():
    r = robot_activo(encoders_ok=False)
    for _ in range(400):
        r.consigna_orugas([0] * N_ORUGAS, [220.0] * N_ORUGAS, True, False)
        r.avanzar(DT)
    assert all(a == 180.0 for a in r.angulo)


def test_sin_encoders_el_incremento_si_funciona():
    """Mover a ciegas se permite; cerrar el lazo no."""
    r = robot_activo(encoders_ok=False)
    for _ in range(50):
        r.consigna_orugas([1, 0, 0, 0], [180.0] * N_ORUGAS, False, False)
        r.avanzar(DT)
    assert r.angulo[0] > 180.0


# --- Bits de error ---------------------------------------------------------

def test_bits_de_error_de_encoders_y_can():
    r = robot_activo(encoders_ok=False, can_ok=False)
    r.consigna_traccion(0.0, 0.0)
    r.avanzar(DT)
    assert r.bits_error & ERR_ENCODER
    assert r.bits_error & ERR_CAN


def test_sin_averias_y_con_consignas_no_hay_errores():
    r = robot_activo()
    r.consigna_traccion(0.0, 0.0)
    r.avanzar(DT)
    assert r.bits_error == 0


# --- Conversion de angulos -------------------------------------------------

def test_fl_y_rr_son_las_espejadas():
    assert [es_espejada(i) for i in range(4)] == [False, True, True, False]


def test_horizontal_es_180_en_las_cuatro():
    assert [elevacion_a_encoder_deg(i, 0.0) for i in range(4)] == [180.0] * 4


def test_vertical_arriba_coincide_con_el_tfg():
    """El TFG documenta {90, 270, 270, 90} para {FR, FL, RR, RL}."""
    noventa = math.pi / 2
    obtenido = [round(elevacion_a_encoder_deg(i, noventa)) for i in range(4)]
    assert obtenido == [90, 270, 270, 90]


def test_la_conversion_va_y_vuelve():
    for i in range(4):
        for rad in (-1.2, -0.3, 0.0, 0.5, 1.4):
            ida = elevacion_a_encoder_deg(i, rad)
            assert math.isclose(encoder_a_elevacion_rad(i, ida), rad, abs_tol=1e-12)


# --- Integracion -----------------------------------------------------------

def test_dt_no_positivo_no_hace_nada():
    r = robot_activo()
    r.consigna_traccion(40.0, 40.0)
    r.avanzar(0.0)
    r.avanzar(-1.0)
    assert r.vel_izq_dps == 0.0
