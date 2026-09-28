"""Tests de sim_core. No necesitan ROS: corren en el PC con pytest."""
import math

from starcrawler_sim.sim_core import (
    GRADOS_POR_CUENTA,
    N_ORUGAS,
    OFFSETS_ENCODER,
    RobotSimulado,
    VEL_ELEVACION_DPS,
    leer_as5600,
    rate_limit,
    ERR_WATCHDOG,
    ERR_CAN,
    ERR_ENCODER,
    ERR_SIMULADO,
)

DT = 0.02


def robot_activo(**kwargs):
    """Robot con una consigna fresca, para que no salte el watchdog."""
    r = RobotSimulado(**kwargs)
    r.consigna_traccion(0.0, 0.0)
    r.consigna_orugas([0] * N_ORUGAS, [180.0] * N_ORUGAS, False, False)
    return r


def mover(r, segundos, incrementos=None, objetivos=None, posicion=False):
    """Manda la misma consigna a 50 Hz, como el teleop."""
    for _ in range(round(segundos / DT)):
        r.consigna_traccion(0.0, 0.0)
        r.consigna_orugas(incrementos or [0] * N_ORUGAS,
                          objetivos or [180.0] * N_ORUGAS, posicion, False)
        r.avanzar(DT)


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


def test_el_watchdog_salta_a_los_300_ms_como_el_firmware():
    r = robot_activo()
    r.avanzar(0.28)
    assert r.seguridad is False
    r.avanzar(0.04)
    assert r.seguridad is True


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
    # 4 dps por ciclo de 10 ms: 8 dps en 20 ms como mucho
    assert 0 < r.vel_izq_dps <= 8.0 + 1e-9


def test_la_traccion_acaba_llegando():
    r = robot_activo()
    for _ in range(100):
        r.consigna_traccion(40.0, -40.0)
        r.avanzar(DT)
    assert math.isclose(r.vel_izq_dps, 40.0, rel_tol=1e-6)
    assert math.isclose(r.vel_der_dps, -40.0, rel_tol=1e-6)


def test_la_traccion_satura_a_40_dps_como_el_firmware():
    r = robot_activo()
    for _ in range(100):
        r.consigna_traccion(100.0, -250.0)
        r.avanzar(DT)
    assert r.vel_izq_dps == 40.0
    assert r.vel_der_dps == -40.0


def test_la_emergencia_para_la_traccion():
    r = robot_activo()
    for _ in range(50):
        r.consigna_traccion(40.0, 40.0)
        r.avanzar(DT)
    assert r.vel_izq_dps > 0
    r.consigna_orugas([0] * N_ORUGAS, [180.0] * N_ORUGAS, False, True)
    r.avanzar(DT)
    assert r.vel_izq_dps == 0.0 and r.vel_der_dps == 0.0


def test_la_emergencia_es_estado_seguro_sin_bit_de_watchdog():
    r = robot_activo()
    r.consigna_orugas([0] * N_ORUGAS, [180.0] * N_ORUGAS, False, True)
    r.avanzar(DT)
    assert r.seguridad is True
    assert not r.bits_error & ERR_WATCHDOG


# --- Elevacion -------------------------------------------------------------

def test_velocidad_de_regimen_del_stepper():
    # 400 pulsos/vuelta, 1:80 y semiperiodo de 1200 us
    assert math.isclose(VEL_ELEVACION_DPS, 4.6875)


def test_el_incremento_mueve_a_la_velocidad_del_stepper():
    r = robot_activo()
    mover(r, 1.0, [1, 0, 0, 0])
    antes = r.angulo[0]
    mover(r, 2.0, [1, 0, 0, 0])
    assert math.isclose((r.angulo[0] - antes) / 2.0, VEL_ELEVACION_DPS,
                        rel_tol=0.01)


def test_la_rampa_arranca_despacio():
    r = robot_activo()
    mover(r, 0.1, [1, 0, 0, 0])
    recorrido = r.angulo[0] - 180.0
    assert 0.0 < recorrido < 0.5 * VEL_ELEVACION_DPS * 0.1


def test_invertir_el_sentido_rehace_la_rampa():
    r = robot_activo()
    mover(r, 1.0, [1, 0, 0, 0])
    arriba = r.angulo[0]
    mover(r, 0.1, [-1, 0, 0, 0])
    assert 0.0 < arriba - r.angulo[0] < 0.5 * VEL_ELEVACION_DPS * 0.1


def test_solo_se_mueve_la_oruga_comandada():
    r = robot_activo()
    mover(r, 1.0, [1, 0, 0, 0])
    assert r.angulo[1] == 180.0 and r.angulo[2] == 180.0 and r.angulo[3] == 180.0


def test_el_incremento_negativo_baja():
    r = robot_activo()
    mover(r, 1.0, [-1, 0, 0, 0])
    assert r.angulo[0] < 180.0


def test_para_en_el_limite_superior():
    r = robot_activo()
    mover(r, 40.0, [1] * N_ORUGAS)          # de sobra para llegar al tope
    assert all(abs(a - 275.0) < 0.1 for a in r.angulo)
    assert all(a >= 275.0 for a in r.angulo_medido)


def test_para_en_el_limite_inferior():
    r = robot_activo()
    mover(r, 40.0, [-1] * N_ORUGAS)
    assert all(abs(a - 85.0) < 0.1 for a in r.angulo)


def test_sin_encoder_no_hay_limites():
    """cc_aplicarLimites deja pasar el comando si el encoder no responde."""
    r = robot_activo(angulos_iniciales=[274.0] * N_ORUGAS, encoders_ok=False)
    mover(r, 2.0, [1, 0, 0, 0])
    assert r.angulo[0] > 275.0


# --- Encoders ----------------------------------------------------------------

def test_el_encoder_cuantiza_como_el_as5600():
    for i in range(N_ORUGAS):
        for angulo in (85.0, 179.99, 180.0, 200.123, 275.0):
            medido = leer_as5600(angulo, OFFSETS_ENCODER[i])
            assert abs(medido - angulo) <= GRADOS_POR_CUENTA / 2 + 1e-9


def test_con_el_encoder_caido_se_publica_el_ultimo_angulo_bueno():
    r = robot_activo(encoders_ok=False)
    mover(r, 1.0, [1, 0, 0, 0])
    assert r.angulo[0] > 180.0
    assert r.angulo_medido[0] == 180.0


# --- Control de posicion ---------------------------------------------------

def test_la_posicion_llega_al_objetivo_y_para():
    r = robot_activo()
    mover(r, 8.0, objetivos=[200.0] * N_ORUGAS, posicion=True)
    # para a 0,5 grados medidos, mas la resolucion del encoder
    assert all(abs(a - 200.0) <= 0.5 + GRADOS_POR_CUENTA for a in r.angulo)


def test_la_posicion_no_arranca_por_menos_de_un_grado():
    r = robot_activo()
    mover(r, 2.0, objetivos=[180.9] * N_ORUGAS, posicion=True)
    assert r.angulo == [180.0] * N_ORUGAS


def test_la_posicion_no_se_pasa_de_largo():
    r = robot_activo()
    mover(r, 3.0, objetivos=[182.0] * N_ORUGAS, posicion=True)
    assert all(181.4 <= a <= 182.0 for a in r.angulo)


def test_un_dt_grande_se_integra_en_ciclos_de_10_ms():
    r = robot_activo()
    r.consigna_orugas([0] * N_ORUGAS, [182.0] * N_ORUGAS, True, False)
    r.avanzar(0.25)               # un paso grande a proposito
    assert all(180.0 < a < 182.0 for a in r.angulo)


def test_sin_encoders_no_hay_lazo_cerrado():
    r = robot_activo(encoders_ok=False)
    mover(r, 8.0, objetivos=[220.0] * N_ORUGAS, posicion=True)
    assert all(a == 180.0 for a in r.angulo)


def test_sin_encoders_el_incremento_si_funciona():
    """Mover a ciegas se permite; cerrar el lazo no."""
    r = robot_activo(encoders_ok=False)
    mover(r, 1.0, [1, 0, 0, 0])
    assert r.angulo[0] > 180.0


# --- Bits de error ---------------------------------------------------------

def test_bits_de_error_de_encoders_y_can():
    r = robot_activo(encoders_ok=False, can_ok=False)
    r.consigna_traccion(0.0, 0.0)
    r.avanzar(DT)
    assert r.bits_error & ERR_ENCODER
    assert r.bits_error & ERR_CAN


def test_sin_averias_y_con_consignas_solo_avisa_de_que_es_simulado():
    r = robot_activo()
    r.consigna_traccion(0.0, 0.0)
    r.avanzar(DT)
    assert r.bits_error == ERR_SIMULADO


# --- Integracion -----------------------------------------------------------

def test_dt_no_positivo_no_hace_nada():
    r = robot_activo()
    r.consigna_traccion(40.0, 40.0)
    r.avanzar(0.0)
    r.avanzar(-1.0)
    assert r.vel_izq_dps == 0.0
