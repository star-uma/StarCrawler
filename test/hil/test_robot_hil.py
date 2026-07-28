"""
test_robot_hil.py
=================
Tests Hardware-In-the-Loop del firmware unificado StarCrawler.

Requieren el robot encendido y conectado a la red "Horu" (o la que toque).
Si el robot no responde, TODOS los tests se saltan automaticamente, asi que
la suite es segura de lanzar en cualquier momento:

    pip install pytest
    cd test/hil
    pytest -v -s

Variables de entorno opcionales:
    ROBOT_IP           (por defecto 192.168.10.101)
    PUERTO_ROBOT       (por defecto 8885)
    PUERTO_TELEMETRIA  (por defecto 8886)

SEGURIDAD: todos los tests envian consignas de velocidad CERO. Aun asi,
en modo 5 (nivelado) el robot PUEDE mover las orugas: mantener el robot
elevado sobre tacos o con los motores de elevacion sin alimentar la
primera vez.
"""
import os
import socket
import struct
import time

import pytest

ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.10.101")
PUERTO_ROBOT = int(os.environ.get("PUERTO_ROBOT", "8885"))
PUERTO_TELEMETRIA = int(os.environ.get("PUERTO_TELEMETRIA", "8886"))

FORMATO = "<9h"  # 9 x int16 little-endian (18 bytes)

# Bits de error de la telemetria (control_core.h)
ERR_ENCODER = 0x0F  # bits 0..3
ERR_IMU = 1 << 4
ERR_CAN = 1 << 5
ERR_WATCHDOG = 1 << 6


def datagrama(modo, vel_izq=0, vel_der=0, o=(0, 0, 0, 0)):
    """Construye el datagrama PC->robot (id=1)."""
    return struct.pack(FORMATO, 1, vel_izq, vel_der, *o, modo, 0)


class Enlace:
    """Socket TX (comandos) + RX (telemetria) hacia el robot."""

    def __init__(self):
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("", PUERTO_TELEMETRIA))
        self.rx.settimeout(0.5)

    def enviar(self, paquete):
        self.tx.sendto(paquete, (ROBOT_IP, PUERTO_ROBOT))

    def recibir_telemetria(self, timeout=1.0):
        """Devuelve la ultima telemetria decodificada o None."""
        fin = time.monotonic() + timeout
        ultima = None
        while time.monotonic() < fin:
            try:
                datos, _ = self.rx.recvfrom(64)
            except socket.timeout:
                continue
            if len(datos) >= 18:
                campos = struct.unpack(FORMATO, datos[:18])
                if campos[0] == 2:  # origen robot
                    ultima = {
                        "angulos": [campos[i] / 100.0 for i in range(1, 5)],
                        "roll": campos[5] / 100.0,
                        "pitch": campos[6] / 100.0,
                        "modo": campos[7],
                        "errores": campos[8] & 0xFFFF,
                    }
        return ultima

    def mantener(self, modo, duracion_s, o=(0, 0, 0, 0)):
        """Envia paquetes a 50 Hz durante 'duracion_s' y devuelve telemetria."""
        fin = time.monotonic() + duracion_s
        ultima = None
        while time.monotonic() < fin:
            self.enviar(datagrama(modo, o=o))
            try:
                datos, _ = self.rx.recvfrom(64)
                if len(datos) >= 18:
                    campos = struct.unpack(FORMATO, datos[:18])
                    if campos[0] == 2:
                        ultima = {
                            "angulos": [campos[i] / 100.0 for i in range(1, 5)],
                            "roll": campos[5] / 100.0,
                            "pitch": campos[6] / 100.0,
                            "modo": campos[7],
                            "errores": campos[8] & 0xFFFF,
                        }
            except socket.timeout:
                pass
            time.sleep(0.02)
        return ultima

    def cerrar(self):
        # Parada limpia: modo 1, velocidad 0, varias veces
        for _ in range(5):
            self.enviar(datagrama(1))
            time.sleep(0.02)
        self.tx.close()
        self.rx.close()


@pytest.fixture(scope="module")
def enlace():
    e = Enlace()
    # ¿Hay robot? Enviamos keepalive 2 s y esperamos telemetria
    tele = e.mantener(1, 2.0)
    if tele is None:
        e.cerrar()
        pytest.skip(f"Robot no accesible en {ROBOT_IP}:{PUERTO_ROBOT} "
                    "(sin telemetria). Tests HIL saltados.")
    yield e
    e.cerrar()


def test_telemetria_formato(enlace):
    """La telemetria llega, con id=2 y campos en rangos fisicos."""
    tele = enlace.mantener(1, 1.0)
    assert tele is not None, "sin telemetria durante 1 s"
    for ang in tele["angulos"]:
        assert -30.0 <= ang <= 390.0, f"angulo fuera de rango fisico: {ang}"
    assert -180.0 <= tele["roll"] <= 180.0
    assert -180.0 <= tele["pitch"] <= 180.0


def test_eco_de_modo(enlace):
    """El modo activo reportado sigue al modo enviado."""
    tele = enlace.mantener(1, 1.0)
    assert tele["modo"] == 1
    tele = enlace.mantener(3, 1.0)  # incremental con vector {0,0,0,0}: quieto
    assert tele["modo"] == 3
    tele = enlace.mantener(1, 1.0)
    assert tele["modo"] == 1


def test_watchdog_activa_y_recupera(enlace):
    """Sin paquetes >500 ms el robot entra en seguridad; luego recupera."""
    enlace.mantener(1, 1.0)
    time.sleep(1.2)  # silencio: debe saltar el watchdog
    tele = enlace.recibir_telemetria(timeout=1.5)
    assert tele is not None, "la telemetria debe seguir llegando en seguridad"
    assert tele["errores"] & ERR_WATCHDOG, "bit de watchdog no activado"

    tele = enlace.mantener(1, 1.5)
    assert tele is not None
    assert not (tele["errores"] & ERR_WATCHDOG), "watchdog no se ha limpiado"


def test_modo2_estable_sin_consigna_nueva(enlace):
    """En modo 2 con la posicion actual como objetivo, nada se mueve."""
    tele = enlace.mantener(1, 1.0)
    objetivos = tuple(int(round(a)) for a in tele["angulos"])
    tele_antes = enlace.mantener(2, 1.0, o=objetivos)
    tele_despues = enlace.mantener(2, 2.0, o=objetivos)
    for a, b in zip(tele_antes["angulos"], tele_despues["angulos"]):
        assert abs(a - b) < 1.5, f"oruga se ha movido sin consigna: {a} -> {b}"


def test_diagnostico(enlace):
    """Informativo: estado de encoders, IMU y CAN (no falla, imprime)."""
    tele = enlace.mantener(1, 1.0)
    e = tele["errores"]
    print("\n--- Diagnostico del robot ---")
    for i, nombre in enumerate(["FR", "FL", "RR", "RL"]):
        estado = "FALLO" if e & (1 << i) else "OK"
        print(f"  Encoder {nombre}: {estado}  ({tele['angulos'][i]:.1f} deg)")
    print(f"  IMU:  {'FALLO' if e & ERR_IMU else 'OK'} "
          f"(roll={tele['roll']:.1f} pitch={tele['pitch']:.1f})")
    print(f"  CAN:  {'FALLO' if e & ERR_CAN else 'OK'}")
    print(f"  Modo: {tele['modo']}")
