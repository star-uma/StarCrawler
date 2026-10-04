# rmd_uart — un RMD por el puerto serie de su placa, sin ESP32

Para las placas MC-X: el conector de 4 pines (5V, TX, RX, GND) con un
adaptador USB-serie (CH340 o similar). Mismo protocolo que por CAN.

Conexión: TX del adaptador al RX de la placa, RX al TX, y GND. El motor
alimentado. **Cierra MYACTUATOR Assistant antes**: solo uno puede usar el puerto.

Necesita `pyserial` (`py -3.12 -m pip install pyserial`).

## Leer el encoder (no mueve)

```powershell
py -3.12 rmd_leer.py COM14 20
```

Muestra el estado y el encoder cada medio segundo durante 20 s. Gira el eje
a mano: si el número no cambia, el sensor no lee o la placa no está calibrada.

## Prueba de velocidad (mueve)

```powershell
py -3.12 rmd_velocidad.py COM14 10 20 40 -20 --seg 8
```

Eje libre. Cada velocidad (grados por segundo del eje de salida) dura 8 s y
saca la velocidad real, su variación y la corriente. Se para sola si la
corriente pasa de 8 A (`--imax`) o hay un error, y apaga el motor al final.
`--id` cambia el motor si no es el 1.

## Resultado de referencia (02-10-2026)

Motor X8-P36-V3 con placa MC-X-500-O V1.4 nueva, en vacío y recién calibrado:

| Consigna | Real | Variación | Corriente |
|---|---|---|---|
| 10 | 9,0 | ±0,8 | 0,1 A |
| 20 | 19,5 | ±0,9 | 0,1 A |
| 40 | 39,8 | ±1,7 | 0,1 A |
| −20 | −19,5 | ±0,7 | 0,1 A |
