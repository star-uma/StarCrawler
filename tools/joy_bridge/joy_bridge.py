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
# Sin ventana con foco, SDL no actualiza el mando si no se le pide
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
# Por Bluetooth, el driver HIDAPI de SDL da el DS4 por desconectado a los
# 0,5 s de abrirlo (y se apaga el LED): se usa DirectInput
os.environ.setdefault("SDL_JOYSTICK_HIDAPI_PS4", "0")
import pygame  # noqa: E402

# Orden crudo del DualShock 4 en Windows con pygame 2. SDL puede abrirlo de
# dos maneras y el orden cambia; se elige por lo que anuncia el mando:
#   HIDAPI      "PS4 Controller",      6 ejes, 16 botones, 0 hats (cruceta = botones)
#   DirectInput "Wireless Controller", 6 ejes, 14 botones, 1 hat  (cruceta = hat)
# Comprobado con --probar; si tu mando difiere, ajusta la tabla.
DISPOSICIONES = {
    "hidapi": {
        "ejes": {"LX": 0, "LY": 1, "RX": 2, "RY": 3, "L2": 4, "R2": 5},
        "botones": {"X": 0, "O": 1, "[]": 2, "T": 3, "share": 4, "PS": 5,
                    "options": 6, "L3": 7, "R3": 8, "L1": 9, "R1": 10,
                    "arriba": 11, "abajo": 12, "izq": 13, "der": 14},
    },
    "directinput": {
        "ejes": {"LX": 0, "LY": 1, "RX": 2, "L2": 3, "R2": 4, "RY": 5},
        "botones": {"[]": 0, "X": 1, "O": 2, "T": 3, "L1": 4, "R1": 5,
                    "L2": 6, "R2": 7, "share": 8, "options": 9, "L3": 10,
                    "R3": 11, "PS": 12},
    },
}
EJES_WIN = DISPOSICIONES["hidapi"]["ejes"]
BOTONES_WIN = DISPOSICIONES["hidapi"]["botones"]


def elegir_disposicion(mando):
    global EJES_WIN, BOTONES_WIN
    nombre = "directinput" if (mando.get_numhats() >= 1
                               or mando.get_numbuttons() <= 14) else "hidapi"
    EJES_WIN = DISPOSICIONES[nombre]["ejes"]
    BOTONES_WIN = DISPOSICIONES[nombre]["botones"]
    return nombre


class Xbox:
    """Mando de Xbox por la API GameController de SDL: se lee por nombre de
    boton, sin depender del orden crudo, que cambia con el driver de Windows.
    A/B/X/Y van donde X/O/[]/T del DS4, LB/RB a L1/R1, View/Menu a
    share/options y el boton Xbox a PS."""

    def __init__(self, indice):
        from pygame._sdl2 import controller
        controller.init()
        self.c = controller.Controller(indice)
        self.joy = pygame.joystick.Joystick(indice)
        self.joy.init()

    def get_name(self):
        return self.joy.get_name()

    def get_instance_id(self):
        return self.joy.get_instance_id()


def abrir(indice=0):
    """El mando y el nombre de su disposicion."""
    joy = pygame.joystick.Joystick(indice)
    joy.init()
    if "xbox" in joy.get_name().lower():
        return Xbox(indice), "gamecontroller (Xbox)"
    return joy, elegir_disposicion(joy)


def leer_xbox(m):
    eje = lambda k: max(-1.0, m.c.get_axis(k) / 32767.0)    # noqa: E731
    bot = lambda k: 1 if m.c.get_button(k) else 0             # noqa: E731
    lt = eje(pygame.CONTROLLER_AXIS_TRIGGERLEFT)              # 0 suelto, 1 a fondo
    rt = eje(pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
    ejes = [-eje(pygame.CONTROLLER_AXIS_LEFTX), -eje(pygame.CONTROLLER_AXIS_LEFTY),
            1.0 - 2.0 * lt,
            -eje(pygame.CONTROLLER_AXIS_RIGHTX), -eje(pygame.CONTROLLER_AXIS_RIGHTY),
            1.0 - 2.0 * rt,
            float(bot(pygame.CONTROLLER_BUTTON_DPAD_LEFT) - bot(pygame.CONTROLLER_BUTTON_DPAD_RIGHT)),
            float(bot(pygame.CONTROLLER_BUTTON_DPAD_UP) - bot(pygame.CONTROLLER_BUTTON_DPAD_DOWN))]
    botones = [bot(pygame.CONTROLLER_BUTTON_A), bot(pygame.CONTROLLER_BUTTON_B),
               bot(pygame.CONTROLLER_BUTTON_Y), bot(pygame.CONTROLLER_BUTTON_X),
               bot(pygame.CONTROLLER_BUTTON_LEFTSHOULDER),
               bot(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER),
               1 if lt > 0.5 else 0, 1 if rt > 0.5 else 0,
               bot(pygame.CONTROLLER_BUTTON_BACK), bot(pygame.CONTROLLER_BUTTON_START),
               bot(pygame.CONTROLLER_BUTTON_GUIDE),
               bot(pygame.CONTROLLER_BUTTON_LEFTSTICK),
               bot(pygame.CONTROLLER_BUTTON_RIGHTSTICK)]
    return [round(v, 3) for v in ejes], botones


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
    mando, disposicion = abrir(0)
    j = mando.joy if isinstance(mando, Xbox) else mando
    print("Mando: %s | ejes %d | botones %d | hats %d | disposicion %s"
          % (j.get_name(), j.get_numaxes(), j.get_numbuttons(),
             j.get_numhats(), disposicion))
    return mando


def probar(mando):
    print("Mueve sticks y pulsa botones. Ctrl+C para salir.")
    try:
        while True:
            pygame.event.pump()
            if isinstance(mando, Xbox):
                # Ya traducido al orden del DS4 que espera el teleop
                ejes, botones = leer_xbox(mando)
                print("\rejes %s  botones %s   " % (ejes, botones), end="", flush=True)
                time.sleep(0.05)
                continue
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
    if isinstance(mando, Xbox):
        return leer_xbox(mando)
    a = lambda n: mando.get_axis(EJES_WIN[n])          # noqa: E731
    b = lambda n: 1 if mando.get_button(BOTONES_WIN[n]) else 0  # noqa: E731

    # Cruceta: botones en Windows, hat (ejes 6/7) en el nodo joy.
    if mando.get_numhats() > 0:
        hx, hy = mando.get_hat(0)          # hx: +1 derecha; hy: +1 arriba
        dpad_x, dpad_y = -float(hx), float(hy)
    elif "arriba" in BOTONES_WIN:
        dpad_x = float(b("izq") - b("der"))
        dpad_y = float(b("arriba") - b("abajo"))
    else:
        dpad_x = dpad_y = 0.0

    ejes = [-a("LX"), -a("LY"), -a("L2"), -a("RX"), -a("RY"), -a("R2"),
            dpad_x, dpad_y]
    botones = [b("X"), b("O"), b("T"), b("[]"), b("L1"), b("R1"),
               b("L2") if "L2" in BOTONES_WIN else (1 if a("L2") > 0.0 else 0),
               b("R2") if "R2" in BOTONES_WIN else (1 if a("R2") > 0.0 else 0),
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
            # El DS4 se duerme por Bluetooth y al volver es otro dispositivo
            # para SDL: sin esto se seguiria leyendo el viejo, todo a cero.
            for ev in pygame.event.get():
                if (ev.type == pygame.JOYDEVICEREMOVED and mando is not None
                        and ev.instance_id == mando.get_instance_id()):
                    mando = None
                    print("Mando desconectado; espero a que vuelva.")
            # El aviso de alta puede llegar antes que el de baja: se reabre
            # en cuanto vuelve a haber un mando, no con el evento
            if mando is None and pygame.joystick.get_count() > 0:
                mando, disposicion = abrir(0)
                print("Mando reconectado: %s | disposicion %s"
                      % (mando.get_name(), disposicion))
            # Sin mando no se envia nada: joy_udp_node pasa a neutro solo
            if mando is not None:
                ejes, botones = leer(mando)
                sock.sendto(json.dumps({"axes": ejes, "buttons": botones}).encode(),
                            (ip, args.puerto))
            time.sleep(periodo)
    except KeyboardInterrupt:
        print("\nCerrado.")


if __name__ == "__main__":
    main()
