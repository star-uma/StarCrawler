"""Ajuste del lazo de velocidad de un RMD a traves de test_motor_diag_simple.

Conduce el sketch por el puerto serie. Ver README.md de esta carpeta.

  python ajuste_rmd.py COM7 estado
  python ajuste_rmd.py COM7 ganancias
  python ajuste_rmd.py COM7 medir --dps 10 20 40 --seg 8
  python ajuste_rmd.py COM7 barrido 100/25 200/50 200/75 --dps 10 --seg 8
  python ajuste_rmd.py COM7 ram 200 50
  python ajuste_rmd.py COM7 rom 200 50 --confirmo
  python ajuste_rmd.py COM7 --id 142 ganancias
"""
import argparse
import re
import statistics as st
import sys
import time

import serial


class Sketch:
    """Habla con test_motor_diag_simple: escribe una orden y lee hasta un texto."""

    def __init__(self, puerto, id_motor):
        self.s = serial.Serial(puerto, 115200, timeout=0.2)
        time.sleep(3.0)                       # abrir el puerto resetea la placa
        self.s.reset_input_buffer()
        if id_motor != 0x141:
            self.orden("i", "hexadecimal", 3)
            r = self.orden("%X" % id_motor, "MOTOR 0x", 3)
            if not any("MOTOR 0x" in x for x in r):
                sys.exit("El sketch no acepto el ID 0x%X" % id_motor)

    def orden(self, texto, hasta, tmax):
        self.s.write((texto + "\n").encode())
        lineas = []
        fin = time.time() + tmax
        while time.time() < fin:
            linea = self.s.readline().decode("utf-8", "replace").rstrip()
            if linea:
                lineas.append(linea)
                if hasta and hasta in linea:
                    break
        return lineas

    def cerrar(self):
        self.s.close()


def leer_ganancias(sk):
    r = sk.orden("g", "V4 idx 8", 4)
    for linea in r:
        m = re.search(r"corriente Kp/Ki (\d+)/(\d+)\s+velocidad Kp/Ki (\d+)/(\d+)\s+posicion Kp/Ki (\d+)/(\d+)", linea)
        if m:
            return tuple(int(x) for x in m.groups())
    sys.exit("No se pudieron leer las ganancias (0x30):\n" + "\n".join(r))


def escribir_ram(sk, kp, ki):
    sk.orden("k", "(0-255)", 3)
    sk.orden(str(kp), "(0-255)", 3)
    r = sk.orden(str(ki), "ESCRITO", 3)
    return any("ESCRITO" in x for x in r)


def grabar_rom(sk, kp, ki):
    sk.orden("r", "(0-255)", 3)
    sk.orden(str(kp), "(0-255)", 3)
    sk.orden(str(ki), "confirmar", 3)
    r = sk.orden("SI", "GRABADO EN ROM", 5)
    return any("GRABADO EN ROM" in x for x in r)


def medir(sk, dps, seg):
    """Consigna fija con angulo fino (0,01 grados); descarta el primer medio segundo."""
    sk.orden("f", "dps", 3)
    sk.orden(str(dps), "segundos", 3)
    r = sk.orden(str(seg), "Motor apagado", seg + 6)
    d = [tuple(int(v) for v in x.split(",")[2:]) for x in r if x.startswith("FIN,")]
    d = [x for x in d if x[0] > 500]
    if len(d) < 10:
        return None
    t = [x[0] / 1000.0 for x in d]
    ang = [x[3] / 100.0 for x in d]
    corr = [x[2] for x in d]
    v = [(ang[k + 1] - ang[k]) / (t[k + 1] - t[k]) for k in range(len(d) - 1) if t[k + 1] > t[k]]
    return dict(media=st.mean(v), desv=st.pstdev(v), vmin=min(v), vmax=max(v),
                paradas=sum(1 for x in v if x < 0.2 * abs(dps)),
                i_desv=st.pstdev(corr), i_max=max(abs(x) for x in corr),
                fallos=sum(1 for x in r if x.startswith("MISS")))


CABECERA = "%-9s %6s %6s %6s %6s %6s %5s %7s %6s" % (
    "", "dps", "media", "desv", "min", "max", "par", "I_desv", "I_max")


def fila(etiqueta, dps, m):
    if m is None:
        return "%-9s %6s  sin datos (motor sin responder?)" % (etiqueta, dps)
    return "%-9s %6s %6.2f %6.2f %6.1f %6.1f %5d %7.1f %6d" % (
        etiqueta, dps, m["media"], m["desv"], m["vmin"], m["vmax"], m["paradas"],
        m["i_desv"], m["i_max"])


def main():
    ap = argparse.ArgumentParser(description="Ajuste del lazo de velocidad de un RMD")
    ap.add_argument("puerto")
    ap.add_argument("--id", default="141", help="ID del motor en hexadecimal (141-148)")
    sub = ap.add_subparsers(dest="orden", required=True)
    sub.add_parser("estado")
    sub.add_parser("ganancias")
    for nombre in ("ram", "rom"):
        p = sub.add_parser(nombre)
        p.add_argument("kp", type=int)
        p.add_argument("ki", type=int)
        if nombre == "rom":
            p.add_argument("--confirmo", action="store_true",
                           help="obligatorio: la ROM sobrevive al apagado")
    p = sub.add_parser("medir")
    p.add_argument("--dps", type=float, nargs="+", default=[10.0])
    p.add_argument("--seg", type=int, default=8)
    p = sub.add_parser("barrido")
    p.add_argument("pares", nargs="+", help="Kp/Ki de velocidad, p. ej. 200/50")
    p.add_argument("--dps", type=float, nargs="+", default=[10.0])
    p.add_argument("--seg", type=int, default=8)
    a = ap.parse_args()

    for n in ("kp", "ki"):
        if hasattr(a, n) and not 0 <= getattr(a, n) <= 255:
            sys.exit("%s fuera de rango (0-255)" % n)
    if a.orden == "rom" and not a.confirmo:
        sys.exit("Grabar en ROM es permanente: repite la orden con --confirmo")

    sk = Sketch(a.puerto, int(a.id, 16))
    try:
        if a.orden == "estado":
            print("\n".join(sk.orden("s", "ang vuelta", 4)))
        elif a.orden == "ganancias":
            g = leer_ganancias(sk)
            print("motor 0x%s  corriente %d/%d  velocidad %d/%d  posicion %d/%d" % ((a.id,) + g))
        elif a.orden == "ram":
            print("RAM: " + ("escrito" if escribir_ram(sk, a.kp, a.ki) else "SIN CONFIRMAR"))
            print("velocidad Kp/Ki ahora %d/%d (se pierde al apagar el motor)" % leer_ganancias(sk)[2:4])
        elif a.orden == "rom":
            print("ROM: " + ("grabado" if grabar_rom(sk, a.kp, a.ki) else "SIN CONFIRMAR"))
            print("velocidad Kp/Ki ahora %d/%d" % leer_ganancias(sk)[2:4])
        elif a.orden == "medir":
            print(CABECERA)
            kp, ki = leer_ganancias(sk)[2:4]
            for dps in a.dps:
                print(fila("%d/%d" % (kp, ki), dps, medir(sk, dps, a.seg)))
                sys.stdout.flush()
        elif a.orden == "barrido":
            original = leer_ganancias(sk)[2:4]
            print("ganancias de partida %d/%d; se restauran al final" % original)
            print(CABECERA)
            try:
                for par in a.pares:
                    kp, ki = (int(x) for x in par.split("/"))
                    if not escribir_ram(sk, kp, ki):
                        print("%-9s  no se pudo escribir en RAM" % par)
                        continue
                    for dps in a.dps:
                        m = medir(sk, dps, a.seg)
                        print(fila(par, dps, m))
                        sys.stdout.flush()
                        if m and (m["i_max"] > 600 or m["desv"] > 0.6 * abs(dps)):
                            print("  >>> corriente u oscilacion excesiva: corto el barrido")
                            return
            finally:
                escribir_ram(sk, *original)
                print("restauradas %d/%d en RAM" % leer_ganancias(sk)[2:4])
    finally:
        sk.orden("p", "apagado", 2)
        sk.cerrar()


if __name__ == "__main__":
    main()
