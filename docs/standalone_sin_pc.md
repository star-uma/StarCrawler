# StarCrawler standalone: robot sin PC

Rama `feature/standalone-sin-pc`. El mando Xbox se conecta por **Bluetooth
directamente al ESP32** (librería Bluepad32): desaparecen el PC, el script
Python y el WiFi. Enciendes el robot, enciendes el mando, y a conducir.

```
ANTES:  [Mando] --BT--> [PC + StarCrawlerXbox.py] --WiFi/UDP--> [ESP32] --> motores
AHORA:  [Mando] --------------Bluetooth----------------------> [ESP32] --> motores
```

## Qué cambia y qué no

| | Con PC (main) | Standalone (esta rama) |
|---|---|---|
| Lectura del mando | pygame en el PC | Bluepad32 en el ESP32 |
| Lógica de modos/mezcla | Python (`StarCrawlerXbox.py`) | `gamepad_core.c` (puerto 1:1, testeado) |
| Transporte | UDP WiFi a 50 Hz | interno (sin red) |
| Watchdog | 500 ms sin paquetes UDP | desconexión BT (instantánea) + 500 ms sin datos frescos |
| Modos | 1–5 | 1–4 (sin IMU; RB cicla 1→2→3→4→1) |
| Control de motores | idéntico | idéntico (mismos módulos que la variante básica) |
| Telemetría | UDP al PC | línea de estado por serie a 1 Hz |

El mapeo del mando es el mismo del README (sticks, RB, A/B/X/Y, cruceta,
gatillos). La lógica portada está verificada con 37 tests unitarios
contrastados caso a caso con el Python (`test/host/test_gamepad_core.cpp`).

## Requisitos

1. **Mando Xbox con Bluetooth de verdad**: One S (modelo 1708) o posterior,
   incluida la serie X/S. Los primeros mandos de Xbox One (1537/1697) usan RF
   propietario y NO valen. Prueba rápida: si se empareja con un móvil, vale.
2. **Toolchain Bluepad32** (board package propio, NO vale el core esp32 normal
   porque Bluepad32 sustituye la pila Bluetooth):

```powershell
arduino-cli core install esp32-bluepad32:esp32 --additional-urls "https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
```

La librería `Bluepad32_ESP32` viene incluida en el board package (no hay que
instalarla aparte). `ACAN2515` sí hace falta: `arduino-cli lib install ACAN2515`.

## Compilar y flashear

```powershell
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_standalone
arduino-cli upload -p COMx --fqbn esp32-bluepad32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_standalone
```

(Los otros dos firmwares se siguen compilando con el core `esp32:esp32` normal.)

## Emparejar el mando

1. Alimenta el robot (el ESP32 queda en modo descubrimiento automáticamente).
2. Enciende el mando y mantén el botón de sincronización (arriba) hasta que la
   luz parpadee rápido.
3. En unos segundos conecta y el monitor serie muestra `[MANDO] Conectado`.
4. El emparejamiento persiste entre reinicios. Para re-emparejar otro mando,
   descomenta `BP32.forgetBluetoothKeys()` en el setup, flashea una vez y
   vuelve a comentarla.

## Seguridad

- **Mando desconectado** (apagado, sin batería, fuera de alcance): parada
  inmediata — RMD liberados, steppers parados. Es más rápido que el watchdog
  UDP del sistema con PC porque la pila Bluetooth notifica la desconexión.
- **Mando conectado pero congelado** (sin datos frescos 500 ms): misma parada.
- Arranque en estado seguro: nada se mueve hasta que el mando conecta.

## Dashboard en tiempo real

El firmware emite por serie una línea `TLM,...` a 10 Hz (modo, velocidades,
4 ángulos, bits de error). `control/StarCrawlerDashboard.py` la pinta en el
navegador con el **robot dibujado en vivo** (las 4 orugas girando con su
ángulo real), velocidades, errores decodificados y gráficas de 60 s:

```powershell
python control\StarCrawlerDashboard.py --serial COM7   # standalone por USB
python control\StarCrawlerDashboard.py --udp           # firmwares WiFi (puerto 8886)
python control\StarCrawlerDashboard.py --demo          # probarlo sin robot
```

Se abre solo en http://localhost:8000. Solo un programa puede tener el puerto
serie abierto: cierra el Serial Monitor antes de lanzarlo con `--serial`
(el propio dashboard hace de monitor: los eventos salen en su consola y en
el panel "Eventos" de la web).

## Limitaciones de esta variante

- Sin telemetría remota (no hay WiFi): la telemetría sale por el puerto serie
  (USB). Si algún día se quiere por red, se puede reactivar el WiFi, pero
  ojo: WiFi y Bluetooth comparten la radio del ESP32 y compiten por ella.
- Sin modo 5 (nivelado): no hay IMU en esta rama.
- Los tests HIL de `test/hil/` no aplican (no hay UDP); los sketches de
  `test/target/` sí (se compilan con el core esp32 normal).
