"""Lee el encoder de un RMD por su puerto serie (protocolo V4.2, trama RS485).

    py -3.12 rmd_leer.py COM14 [segundos] [id]

Trama: 3E id 08 d0..d7 CRC16 (Modbus). Solo lecturas: 0x90 y 0x9A.
"""
import struct
import sys
import time

import serial


def crc16(datos: bytes) -> int:
    crc = 0xFFFF
    for b in datos:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def pedir(s, ident, cmd):
    cuerpo = bytes([0x3E, ident, 0x08, cmd, 0, 0, 0, 0, 0, 0, 0])
    s.reset_input_buffer()
    s.write(cuerpo + struct.pack('<H', crc16(cuerpo)))
    r = s.read(13)
    if len(r) == 13 and r[0] == 0x3E and r[3] == cmd and crc16(r[:11]) == struct.unpack('<H', r[11:])[0]:
        return r[3:11]
    return None


puerto = sys.argv[1]
seg = float(sys.argv[2]) if len(sys.argv) > 2 else 20
ident = int(sys.argv[3]) if len(sys.argv) > 3 else 1
s = serial.Serial(puerto, 115200, timeout=0.1)
e = pedir(s, ident, 0x9A)
if e is None:
    print('sin respuesta del motor %d en %s' % (ident, puerto))
    sys.exit(1)
print('estado: temp %d C, tension %.1f V, error 0x%04X' % (
    struct.unpack('b', e[1:2])[0], struct.unpack('<H', e[4:6])[0] * 0.1,
    struct.unpack('<H', e[6:8])[0]))
t0 = time.time()
while time.time() - t0 < seg:
    d = pedir(s, ident, 0x90)
    if d:
        enc, bruto, off = struct.unpack('<HHH', d[2:8])
        print('t=%4.1f  encoder %5d  bruto %5d  offset %5d' % (time.time() - t0, enc, bruto, off), flush=True)
    else:
        print('t=%4.1f  sin respuesta' % (time.time() - t0), flush=True)
    time.sleep(0.5)
s.close()
