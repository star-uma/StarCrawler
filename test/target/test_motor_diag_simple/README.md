# test_motor_diag_simple — diagnóstico y ajuste de un motor RMD

Un solo motor RMD (protocolo V3) en el bus CAN. Sirve para ver cómo gira de
verdad y para ajustar su lazo de velocidad.

| Placa | CAN |
|---|---|
| ESP32 DevKit V1 + TJA1050 | TWAI interno, TX GPIO5 / RX GPIO35 (los de `test_can_esp32`) |
| MKR WiFi 1010 + MKR CAN Shield | librería `CAN` (sandeepmistry) |

## Compilar y flashear

Desde `test/target`, en un solo paso (compilar para una placa y subir por
separado a la otra mezcla la caché y el `upload` falla):

```powershell
arduino-cli compile -u -p COM7 --fqbn esp32:esp32:esp32doit-devkit-v1 test_motor_diag_simple
arduino-cli compile -u -p COMX --fqbn arduino:samd:mkrwifi1010 test_motor_diag_simple
arduino-cli monitor -p COM7 --config 115200
```

## Comandos del sketch

| | |
|---|---|
| `s` | estado: tensión, flags, temperatura, ángulos (no mueve) |
| `c` | consigna de velocidad y tiempo, CSV `DAT` con el ángulo de 1° (mueve) |
| `f` | igual, pero pide también el ángulo multivuelta de 0,01° (mueve) |
| `g` | leer las ganancias PID (`0x30`) |
| `k` | escribir Kp/Ki de velocidad en **RAM** (`0x31`): se pierde al apagar el motor |
| `r` | grabar Kp/Ki de velocidad en **ROM** (`0x32`): permanente, pide `SI` |
| `i` | elegir motor (ID 141–148) |
| `p` | parar |

## `ajuste_rmd.py`

Conduce el sketch y hace las cuentas. Necesita `pyserial`.

```powershell
python ajuste_rmd.py COM7 ganancias
python ajuste_rmd.py COM7 medir --dps 10 20 40 --seg 8
python ajuste_rmd.py COM7 barrido 100/25 200/50 200/75 --dps 10 --seg 8
python ajuste_rmd.py COM7 ram 200 50
python ajuste_rmd.py COM7 rom 200 50 --confirmo
python ajuste_rmd.py COM7 --id 142 ganancias
```

`medir` da la velocidad real a partir del ángulo de 0,01° (la que devuelve el
motor va en escalones de 3 dps y no sirve para esto): media, desviación,
mínimo, máximo, micro-paradas (<20 % de la consigna) y corriente. El `barrido`
prueba en RAM, corta si la corriente pasa de 6 A o la oscilación se dispara, y
al final deja las ganancias con las que empezó.

## Motor 0x141 (25-09-2026)

A baja velocidad iba a tirones: con las ganancias de fábrica (velocidad
Kp/Ki 100/25), a 10 dps la velocidad real oscilaba ±2,2 dps con 0–5
micro-paradas por tanda de 8 s. La posición media sí era lineal.

| Consigna | Fábrica 100/25 | 200/50 (grabado en ROM) |
|---|---|---|
| 10 dps | desv ~2,2, 0–5 paradas | desv 1,0–1,2, 0–2 paradas |
| 20 dps | desv 0,9–1,6, picos a 31 | desv ~0,6 |
| 40 dps | desv 0,9–1,4 | desv 0,7–1,3 |

200/75 mejora algo más a 10 dps pero mete valles a 40. A 10 dps sigue
quedando alguna micro-parada suelta: no pedir tracción por debajo de ~15 dps.

Pendiente: los motores 0x142–0x144 con `--id`.
