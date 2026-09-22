"""Puente del mando: Windows (pygame) -> UDP -> nodo joy_udp_node en el WSL.

WSL no ve el mando, ni por Bluetooth ni por USB. Este script lo lee en
Windows y lo manda por UDP en el mismo orden y signo que publica el nodo
`joy` de ROS 2 en Linux con un DualShock 4 (driver hid-sony), para que
ds4.yaml y el teleop no distingan una fuente de otra:

  ejes    0 LX  1 LY  2 L2  3 RX  4 RY  5 R2  6 cruceta X  7 cruceta Y
          (izquierda y arriba = +1; gatillos: +1 suelto, -1 a fondo)
  botones 0 X  1 O  2 T  3 []  4 L1  5 R1  6 L2  7 R2  8 share  9 options
          10 PS  11 L3  12 R3

Uso:
  py -3.12 joy_bridge.py                 # envia al WSL (IP via `wsl hostname -I`)
  py -3.12 joy_bridge.py --ip 172.29.1.2 --puerto 8890
  py -3.12 joy_bridge.py --probar        # muestra ejes/botones crudos de pygame
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame  # noqa: E402

# Orden crudo del DualShock 4 en Windows con pygame 2 (SDL2 + HIDAPI).
# Comprobado con --probar; si tu mando difiere, ajusta aqui.
EJES_WIN = {"LX": 0, "LY": 1, "RX": 2, "RY": 3, "L2": 4, "R2": 5}
BOTONES_WIN = {"X": 0, "O": 1, "[]": 2, "T": 3, "share": 4, "PS": 5,
               "options": 6, "L3": 7, "R3": 8, "L1": 9, "R1": 10,
               "arriba": 11, "abajo": 12, "izq": 13, "der": 14}

FRECUENCIA_HZ = 50


def ip_del_wsl():
    try:
        salida = subprocess.check_output(["wsl", "hostname", "-I"], text=True)
        return salida.split()[0]
    except Exception:
        return None


def abrir_mando():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No hay ningun mando. Empareja el DS4 por Bluetooth (Share+PS) "
                 "o conectalo por USB y vuelve a lanzar.")
    mando = pygame.joystick.Joystick(0)
    mando.init()
    print("Mando: %s | ejes %d | botones %d | hats %d"
          % (mando.get_name(), mando.get_numaxes(), mando.get_numbuttons(),
             mando.get_numhats()))
    return mando


def probar(mando):
    print("Mueve sticks y pulsa botones. Ctrl+C para salir.")
    try:
        while True:
            pygame.event.pump()
            ejes = ["%+.2f" % mando.get_axis(i) for i in range(mando.get_numaxes())]
            botones = [i for i in range(mando.get_numbuttons()) if mando.get_button(i)]
            hats = [mando.get_hat(i) for i in range(mando.get_numhats())]
            print("\rejes %s  botones %-12s hats %s   " % (" ".join(ejes), botones, hats),
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def leer(mando):
    """Devuelve (ejes, botones) ya en el formato del nodo joy de Linux."""
    pygame.event.pump()
    a = lambda n: mando.get_axis(EJES_WIN[n])          # noqa: E731
    b = lambda n: 1 if mando.get_button(BOTONES_WIN[n]) else 0  # noqa: E731

    # Cruceta: botones en Windows, hat (ejes 6/7) en el nodo joy.
    if mando.get_numhats() > 0:
        hx, hy = mando.get_hat(0)          # hx: +1 derecha; hy: +1 arriba
        dpad_x, dpad_y = -float(hx), float(hy)
    else:
        dpad_x = float(b("izq") - b("der"))
        dpad_y = float(b("arriba") - b("abajo"))

    ejes = [-a("LX"), -a("LY"), -a("L2"), -a("RX"), -a("RY"), -a("R2"),
            dpad_x, dpad_y]
    botones = [b("X"), b("O"), b("T"), b("[]"), b("L1"), b("R1"),
               1 if a("L2") > 0.0 else 0, 1 if a("R2") > 0.0 else 0,
               b("share"), b("options"), b("PS"), b("L3"), b("R3")]
    return [round(v, 3) for v in ejes], botones


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ip", help="IP del WSL (por defecto: wsl hostname -I)")
    ap.add_argument("--puerto", type=int, default=8890)
    ap.add_argument("--probar", action="store_true",
                    help="mostrar ejes y botones crudos de pygame")
    args = ap.parse_args()

    mando = abrir_mando()
    if args.probar:
        probar(mando)
        return

    ip = args.ip or ip_del_wsl()
    if not ip:
        sys.exit("No puedo averiguar la IP del WSL; pasala con --ip")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Enviando a %s:%d a %d Hz. Ctrl+C para salir." % (ip, args.puerto, FRECUENCIA_HZ))

    periodo = 1.0 / FRECUENCIA_HZ
    try:
        while True:
            ejes, botones = leer(mando)
            sock.sendto(json.dumps({"axes": ejes, "buttons": botones}).encode(),
                        (ip, args.puerto))
            time.sleep(periodo)
    except KeyboardInterrupt:
        print("\nCerrado.")


if __name__ == "__main__":
    main()
