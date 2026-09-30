"""Tests del avance de contacto_core (§5.6): paso() a 50 Hz sobre escenas de
terreno_prueba, con los numeros del prototipo 2D. Sin ROS.

Convenio REP-103: cabeceo + = morro abajo. Las tablas del prototipo daban el
cabeceo con el morro arriba positivo; aqui van con el signo de REP-103."""
import math

import pytest

from starcrawler_odometry.odometry_core import integrar, normalizar_angulo
from starcrawler_sim import contacto_core as cc
from terreno_prueba import (DT, GEO_PRUEBA, L, R1, R2, V, Terreno, bajada, escalera,
                            escalon, llano, morro, mover_brazos, pared, rampa)

g = math.radians


def simular(ter, grados, segundos, v=V, w=0.0, x=0.0, y=0.0, yaw=0.0, guion=None,
            estado=None):
    """Estados a 50 Hz. guion(estado, t) -> objetivo de los brazos en grados, o None."""
    if estado is None:
        estado = cc.colocar(x, y, yaw, [g(a) for a in grados], ter, GEO_PRUEBA)
    e = estado
    q = e.elevaciones
    objetivo = q
    tray = [e]
    for k in range(round(segundos / DT)):
        if guion is not None:
            nuevo = guion(e, k * DT)
            if nuevo is not None:
                objetivo = tuple(g(a) for a in nuevo)
        q = mover_brazos(q, objetivo)
        e = cc.paso(e, v, w, q, DT, ter, GEO_PRUEBA)
        tray.append(e)
    return tray


def parada(q_grados, cara=1.0):
    """Donde se para la base con la polea pasiva contra la cara (§5.6)."""
    return cara - (0.30 + L * math.cos(g(q_grados)) + R2)


def cabeceo_max(tray):
    return max(math.degrees(e.pose.cabeceo) for e in tray)


def cabeceo_min(tray):
    return min(math.degrees(e.pose.cabeceo) for e in tray)


# --- Llano ---------------------------------------------------------------

def test_en_llano_avanza_como_la_odometria():
    ter = llano()
    e = cc.colocar(0.0, 0.0, 0.0, (0.0,) * 4, ter, GEO_PRUEBA)
    x = y = th = 0.0
    for _ in range(1000):
        e = cc.paso(e, 0.053, 0.2, (0.0,) * 4, DT, ter, GEO_PRUEBA)
        x, y, th = integrar(x, y, th, 0.053, 0.2, DT)
    p = e.pose
    assert abs(p.x - x) < 1e-9 and abs(p.y - y) < 1e-9
    assert abs(normalizar_angulo(p.yaw - th)) < 1e-9
    assert p.z == pytest.approx(R1, abs=1e-12)
    assert p.cabeceo == 0.0 and p.balanceo == 0.0
    assert e.bloqueado is None and not e.sin_traccion and not e.cayendo


# --- Primer contacto -------------------------------------------------------

Q_PRIMER = (0, 10, 20, 30, 45, 60, 69)


@pytest.mark.parametrize('q', Q_PRIMER)
def test_un_canto_5_mm_por_debajo_del_maximo_se_sube(q):
    """El morro gana 1 cm en 2 s. Con 69 grados solo 4 mm: el tramo inferior va
    a 74,56 grados y pasa de 75 en cuanto el morro sube 0,44; ahi se atasca."""
    h = cc.altura_trepable(g(q), GEO_PRUEBA) - 0.005
    tray = simular(escalon(h), (q, q, 0, 0), 2.4, x=parada(q) - 0.02)
    assert morro(tray[-1]) - morro(tray[0]) >= (0.004 if q == 69 else 0.01)


@pytest.mark.parametrize('q', Q_PRIMER)
def test_un_canto_5_mm_por_encima_del_maximo_para_el_morro(q):
    h = cc.altura_trepable(g(q), GEO_PRUEBA) + 0.005
    tray = simular(escalon(h), (q, q, 0, 0), 3.0, x=parada(q) - 0.02)
    e = tray[-1]
    assert e.pose.x == pytest.approx(parada(q), abs=1e-3)
    assert e.bloqueado in ('FR', 'FL')
    assert e.pose.z == pytest.approx(R1, abs=1e-6)
    # Lo que choca queda en los contactos, de pared y a la altura del canto
    assert any(c.tipo == 'pared' and c.pieza in ('FR', 'FL')
               and c.z == pytest.approx(h) for c in e.contactos)


def test_con_72_grados_sube_5_cm_y_no_6_2():
    """El tramo inferior pasa de 75 grados: solo trepa la polea activa."""
    arranque = 1.0 - 0.30 - math.sqrt(R1 ** 2 - (R1 - 0.05) ** 2) - 0.02
    sube = simular(escalon(0.050), (72, 72, 0, 0), 2.4, x=arranque)
    assert morro(sube[-1]) - morro(sube[0]) >= 0.01
    no = simular(escalon(0.062), (72, 72, 0, 0), 3.0, x=arranque)
    assert no[-1].bloqueado in ('FR', 'FL')
    assert no[-1].pose.z == pytest.approx(R1, abs=1e-6)


# --- Paredes ---------------------------------------------------------------

@pytest.mark.parametrize('q', (-20, 0, 45, 90))
def test_contra_una_pared_de_1_m_no_pasa_y_las_orugas_patinan(q):
    tray = simular(pared(1.0, 1.0), (q, q, 0, 0), 20.0, x=0.1)
    llega = next(i for i, e in enumerate(tray) if e.bloqueado)
    tocando = tray[llega:llega + 501]                 # 10 s empujando
    assert len(tocando) == 501
    xs = [e.pose.x for e in tocando]
    assert max(xs) - min(xs) <= 1e-3
    assert sum(e.propuesto for e in tocando[1:]) == pytest.approx(0.053 * 10, abs=0.01)
    assert all(e.bloqueado is not None for e in tocando[1:])
    assert all(c.holgura >= -1e-4 for e in tocando for c in e.contactos)


def test_girar_en_el_sitio_con_el_costado_junto_a_una_pared():
    """El costado a 5 mm de una pared de 0,5 m: no puede girar."""
    ter = Terreno().caja(-2.0, 2.0, 0.5, y0=0.24 + 0.04 + 0.005, y1=1.0)
    for w in (0.2, -0.2):
        tray = simular(ter, (0, 0, 0, 0), 5.0, v=0.0, w=w)
        e = tray[-1]
        assert abs(math.degrees(e.pose.yaw)) < 1.0
        assert e.pose.z == pytest.approx(R1, abs=1e-6)
        assert all(c.holgura >= -1e-4 for s in tray for c in s.contactos)
        assert e.bloqueado is not None


def test_bajar_un_brazo_contra_una_pared_empuja_la_base():
    ter = pared(1.0, 1.0)
    tray = simular(ter, (80, 80, 0, 0), 6.0, x=0.5)
    antes = tray[-1]
    assert antes.bloqueado in ('FR', 'FL')
    tray = simular(ter, None, 14.0, v=0.0, estado=antes,
                   guion=lambda e, t: (20, 20, 0, 0))
    retroceso = tray[-1].pose.x - antes.pose.x
    assert retroceso == pytest.approx(-L * (math.cos(g(20)) - math.cos(g(80))), abs=2e-3)
    assert round(-retroceso, 4) == pytest.approx(0.2758, abs=2e-3)


# --- Tablas del prototipo: escalon de 0,20 m ---------------------------------

@pytest.mark.parametrize('q', (30, 40, 50))
def test_el_escalon_de_20_cm_se_sube_con_los_brazos_entre_30_y_50(q):
    tray = simular(escalon(0.20), (q, q, 0, 0), 32.0, x=0.25)
    e = tray[-1]
    assert e.pose.x > 1.75
    assert e.pose.z == pytest.approx(0.20 + R1, abs=1e-3)
    assert abs(math.degrees(e.pose.cabeceo)) < 0.1
    assert -21.0 < cabeceo_min(tray) < -18.0          # el morro arriba ~19 grados


def test_el_escalon_de_20_cm_con_20_grados_para_en_0_320():
    e = simular(escalon(0.20), (20, 20, 0, 0), 3.0, x=0.25)[-1]
    assert e.pose.x == pytest.approx(0.320, abs=1e-3)
    assert e.bloqueado in ('FR', 'FL')


def test_el_escalon_de_20_cm_con_60_grados_se_atasca_a_media_subida():
    e = simular(escalon(0.20), (60, 60, 0, 0), 20.0, x=0.25)[-1]
    assert math.degrees(e.pose.cabeceo) == pytest.approx(-9.6, abs=1.0)
    assert e.bloqueado in ('FR', 'FL')
    assert e.pose.z < 0.20


def test_el_escalon_de_35_cm_solo_se_sube_coordinando_los_brazos():
    fijo = simular(escalon(0.35), (60, 60, 0, 0), 30.0, x=0.25)
    assert fijo[-1].pose.z < 0.35

    def guion(e, t):
        return (20, 20, 0, 0) if e.pose.cabeceo < g(-8) else None

    tray = simular(escalon(0.35), (60, 60, 0, 0), 45.0, x=0.25, guion=guion)
    assert tray[-1].pose.z == pytest.approx(0.35 + R1, abs=1e-3)
    assert abs(math.degrees(tray[-1].pose.cabeceo)) < 0.1


# --- Bajadas ---------------------------------------------------------------

@pytest.mark.parametrize('h, cabeceo', [(0.10, 9.5), (0.20, 18.3), (0.30, 27.6)])
def test_las_bajadas_se_bajan_con_el_morro_abajo(h, cabeceo):
    tray = simular(bajada(h), (0, 0, 0, 0), 40.0, x=0.3)
    e = tray[-1]
    assert e.pose.x > 2.2
    assert e.pose.z == pytest.approx(R1, abs=1e-4)
    assert abs(e.pose.cabeceo) < 1e-5
    assert cabeceo_max(tray) == pytest.approx(cabeceo, abs=1.5)
    velocidad = max(abs(b.pose.cabeceo - a.pose.cabeceo) for a, b in zip(tray, tray[1:])) / DT
    assert velocidad <= g(90) * 1.01
    # Regresion 1 de §5.8: sin traccion solo mientras vuelca sobre el canto,
    # nunca 'en equilibrio' sobre la panza
    assert not any(s.sin_traccion and not s.cayendo for s in tray)


def test_al_final_de_la_bajada_no_hay_ciclo():
    """Regresion 2 de §5.8: con vuelco a paso fijo el prototipo avanzaba 1 mm
    y retrocedia 0,64 en cada tick al final de la bajada."""
    tray = simular(bajada(0.20), (0, 0, 0, 0), 40.0, x=0.3)
    dx = [b.pose.x - a.pose.x for a, b in zip(tray, tray[1:])]
    cambios = sum(1 for a, b in zip(dx, dx[1:]) if a * b < 0.0 and abs(b) > 1e-6)
    assert cambios <= 2
    # Ya en el suelo, avanza como las orugas
    abajo = [i for i, e in enumerate(tray) if e.pose.z < R1 + 1e-6 and e.pose.x > 1.5]
    assert abajo
    for a, b in zip(tray[abajo[0]:], tray[abajo[0] + 1:]):
        assert b.pose.x - a.pose.x == pytest.approx(V * DT, abs=1e-9)


# --- Escalera (0,20 / 0,25) ------------------------------------------------

def _guion_escalera(delante, detras):
    hecho = []

    def guion(e, t):
        if not hecho and e.pose.cabeceo < g(-15):
            hecho.append(t)
            return (delante, delante, detras, detras)
        return None
    return guion


@pytest.mark.parametrize('delante, detras, sube', [(-6, -6, True), (-15, -15, True),
                                                   (-6, 0, False)])
def test_la_escalera_se_sube_alineando_los_cuatro_brazos(delante, detras, sube):
    tray = simular(escalera(4, 0.20, 0.25), (45, 45, 0, 0), 70.0, x=0.25,
                   guion=_guion_escalera(delante, detras))
    arriba = tray[-1].pose.z > 0.80
    assert arriba == sube


# --- Rampas lisas ----------------------------------------------------------

def test_la_rampa_lisa_de_30_grados_se_sube():
    tray = simular(rampa(30.0, 0.8), (0, 0, 0, 0), 50.0, x=0.25)
    e = tray[-1]
    assert e.pose.x > 1.0 + 0.8 + 0.7                # entero en la meseta
    assert e.pose.z == pytest.approx(0.8 * math.tan(g(30)) + R1, abs=1e-3)
    assert cabeceo_min(tray) == pytest.approx(-30.0, abs=0.5)


def test_la_rampa_lisa_de_45_grados_no_se_sube():
    """Mas de 40 grados sin cantos: las orugas no traccionan."""
    e = simular(rampa(45.0, 1.5), (0, 0, 0, 0), 45.0, x=0.25)[-1]
    assert e.pose.x < 1.0 + 1.5 - 0.3
    assert e.sin_traccion


# --- colocar() y determinismo ------------------------------------------------

def test_colocar_un_robot_mal_apoyado_no_lo_teletransporta():
    """La punta 2 cm dentro de un escalon de 30 cm: el empuje no pasa de 5 mm."""
    x0 = parada(0) + 0.02
    e = cc.colocar(x0, 0.0, 0.0, (0.0,) * 4, escalon(0.30), GEO_PRUEBA)
    assert abs(e.pose.x - x0) <= 0.005 + 1e-6
    assert e.pose.y == 0.0 and e.pose.yaw == 0.0


def test_paso_con_dt_o_con_dos_medios_dt_da_lo_mismo():
    tray = simular(escalon(0.20), (40, 40, 0, 0), 30.0, x=0.25)
    q = tray[0].elevaciones
    for e in tray[100::150]:
        a = cc.paso(e, V, 0.0, q, DT, escalon(0.20), GEO_PRUEBA)
        ter = escalon(0.20)
        b = cc.paso(cc.paso(e, V, 0.0, q, DT / 2, ter, GEO_PRUEBA), V, 0.0, q, DT / 2,
                    ter, GEO_PRUEBA)
        assert a.pose.x == pytest.approx(b.pose.x, abs=1e-3)
        assert a.pose.z == pytest.approx(b.pose.z, abs=1e-3)
        assert math.degrees(a.pose.cabeceo) == pytest.approx(math.degrees(b.pose.cabeceo),
                                                             abs=0.1)


def test_volcado_se_congela_hasta_colocar():
    ter = llano()
    e = cc.colocar(0.0, 0.0, 0.0, (0.0,) * 4, ter, GEO_PRUEBA)
    from dataclasses import replace
    e = replace(e, volcado=True, pose=replace(e.pose, cabeceo=g(80)))
    f = cc.paso(e, V, 0.0, (0.0,) * 4, DT, ter, GEO_PRUEBA)
    assert f.volcado and f.pose == e.pose and f.avance == 0.0
    assert not cc.colocar(0.0, 0.0, 0.0, (0.0,) * 4, ter, GEO_PRUEBA).volcado
