"""Prueba de velocidad de un RMD por su puerto serie (protocolo V4.2, trama RS485).

    py -3.12 rmd_velocidad.py COM14 10 20 40 -20 [--seg 8] [--id 1] [--imax 8]

Para cada consigna (dps del eje de salida) manda 0xA2 cada ~20 ms y lee el
angulo multivuelta (0x92, 0,01 grados) para sacar la velocidad real. Corta si
la corriente pasa de imax o hay error, y al final apaga el motor (0x80).
"""
import argparse
import statistics
import struct
import time

import serial


def crc16(datos: bytes) -> int:
    crc = 0xFFFF
    for b in datos:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def trama(s, ident, datos8):
    cuerpo = bytes([0x3E, ident, 0x08]) + bytes(datos8)
    s.reset_input_buffer()
    s.write(cuerpo + struct.pack('<H', crc16(cuerpo)))
    r = s.read(13)
    if (len(r) == 13 and r[0] == 0x3E and r[3] == datos8[0]
            and crc16(r[:11]) == struct.unpack('<H', r[11:])[0]):
        return r[3:11]
    return None


def apagar(s, ident):
    for _ in range(3):
        if trama(s, ident, [0x80, 0, 0, 0, 0, 0, 0, 0]):
            return True
    return False


ap = argparse.ArgumentParser()
ap.add_argument('puerto')
ap.add_argument('dps', type=float, nargs='+')
ap.add_argument('--seg', type=float, default=8.0)
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--imax', type=float, default=8.0)
a = ap.parse_args()

s = serial.Serial(a.puerto, 115200, timeout=0.05)
e = trama(s, a.id, [0x9A, 0, 0, 0, 0, 0, 0, 0])
if e is None:
    raise SystemExit('el motor %d no contesta en %s' % (a.id, a.puerto))
err = struct.unpack('<H', e[6:8])[0]
print('antes: temp %d C, tension %.1f V, error 0x%04X' % (
    struct.unpack('b', e[1:2])[0], struct.unpack('<H', e[4:6])[0] * 0.1, err))
if err:
    raise SystemExit('el motor tiene un error activo: no lo muevo')

print('%8s %8s %7s %7s %7s %8s %8s' % ('dps', 'media', 'desv', 'min', 'max', 'I_media', 'I_max'))
try:
    for dps in a.dps:
        v = int(round(dps * 100))
        orden = [0xA2, 0, 0, 0] + list(struct.pack('<i', v))
        muestras, corrientes = [], []
        t0 = time.time()
        ant = None
        cortado = None
        while time.time() - t0 < a.seg:
            r = trama(s, a.id, orden)
            g = trama(s, a.id, [0x92, 0, 0, 0, 0, 0, 0, 0])
            ahora = time.time()
            if r:
                iq = struct.unpack('<h', r[2:4])[0] * 0.01
                corrientes.append(abs(iq))
                if abs(iq) > a.imax:
                    cortado = 'corriente %.1f A' % iq
                    break
            if g:
                ang = struct.unpack('<i', g[4:8])[0] * 0.01
                if ant is not None and ahora - t0 > 1.0:     # sin el arranque
                    dt = ahora - ant[1]
                    if dt > 0:
                        muestras.append((ang - ant[0]) / dt)
                ant = (ang, ahora)
            time.sleep(0.01)
        if muestras:
            print('%8.1f %8.2f %7.2f %7.1f %7.1f %8.2f %8.2f' % (
                dps, statistics.mean(muestras), statistics.pstdev(muestras),
                min(muestras), max(muestras),
                statistics.mean(corrientes) if corrientes else float('nan'),
                max(corrientes) if corrientes else float('nan')))
        else:
            print('%8.1f   sin datos de angulo' % dps)
        if cortado:
            print('CORTADO: ' + cortado)
            break
        trama(s, a.id, [0xA2, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.8)
finally:
    print('motor apagado' if apagar(s, a.id) else 'OJO: no confirma el apagado')
    e = trama(s, a.id, [0x9A, 0, 0, 0, 0, 0, 0, 0])
    if e:
        print('despues: temp %d C, error 0x%04X' % (
            struct.unpack('b', e[1:2])[0], struct.unpack('<H', e[6:8])[0]))
    s.close()
