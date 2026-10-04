# test_rmd_detectar — qué motor RMD hay en el bus

Para un motor nuevo o una placa controladora cambiada. Busca el motor y lo lee
entero. Solo la orden `t` lo mueve, y poco.

| Placa | CAN |
|---|---|
| ESP32 DevKit V1 + transceptor | TX GPIO5 / RX GPIO35 (como `test_can_esp32`) |

## Compilar y subir

Desde `test/target`, en un solo paso:

```powershell
arduino-cli compile -u -p COM7 --fqbn esp32:esp32:esp32doit-devkit-v1 test_rmd_detectar
arduino-cli monitor -p COM7 --config 115200
```

Cambia `COM7` por el puerto del ESP32 (`arduino-cli board list`).

## Órdenes

| | ¿Mueve? | Qué hace |
|---|---|---|
| `d` | No | Busca motores en los ID 1–32, a 1 Mbps y a 500 kbps, con modelo, versión y estado |
| `l` | No | Lo lee todo del motor 1: estado, corriente de cada fase, encoder en bruto, cero, PID y aceleraciones |
| `e` | No | Encoder en directo 20 s, para girar el eje a mano y ver si el número cambia |
| `b` / `B` | No | Suelta / echa el freno (si el motor no tiene, no hace nada) |
| `t` | **Sí, poco** | 1,5 A durante 1 s, con el encoder y las tres fases; apaga al final |

## Cómo leerlo

- **Encoder en bruto a 0 fijo**, aunque gire el eje: placa sin calibrar o
  sensor sin señal.
- **"No Adjust" o cero en 1000**: la placa no está calibrada. Se calibra con
  MYACTUATOR Assistant 3.0 (pantalla *Motor Adjust* → *Adjust Encoder*).
- **Error 0x0002 (bloqueo)**: empuja y no gira. Mirar primero que gire suelto
  a mano: una placa mal asentada o un tornillo largo lo frenan.

Este firmware guarda el PID en otro formato: **no uses `k` ni `r` de
`test_motor_diag_simple`** con él.
