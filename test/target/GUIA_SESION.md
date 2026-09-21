# Guía de sesión — puesta en marcha

Hoja de ruta para una sesión con el robot delante. El detalle de cada prueba
está en [README.md](README.md); esto es el resumen para tener al lado.

---

## Las tres reglas

1. **Robot sobre tacos** antes de cualquier cosa que se mueva (pasos 4 y 5).
2. **Nunca dos placas mandando en el bus CAN.** Si usas el MKR, el ESP32 no
   puede estar transmitiendo por CAN — desconéctalo del bus o no lo alimentes.
3. **Un "no responde" es un resultado**, no un fracaso. Apúntalo igual.

---

## Antes de ir al robot

```powershell
git fetch && git checkout feature/test-target-bringup
.\test\target\instalar_entorno.ps1
```

El script compila todos los sketches al terminar. Si algo falla, que falle
aquí y no con el robot delante.

Para saber en qué puerto está cada placa:

```powershell
arduino-cli board list
```

---

## Paso 1 — Encoders · ESP32 · no se mueve nada

```powershell
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test\target\test_encoders
arduino-cli monitor -p COMx -c baudrate=115200
```

1. `s` — ¿responden los 4? Si falla el multiplexor, para y arréglalo: sin él
   no se lee ningún encoder.
2. `c` — mueve cada brazo **a mano** y mira qué columna cambia.
3. Pon las cuatro orugas **horizontales** → `o` → confirma con `SI` → copia la
   línea `#define OFFSETS_ENCODER {...}` que imprime.

| Canal | Oruga que se mueve de verdad | Offset medido |
|---|---|---|
| 0 | | |
| 1 | | |
| 2 | | |
| 3 | | |

## Paso 2 — IMU · ESP32 · no se mueve nada

**Sáltatelo si la IMU sigue cableada al MKR.** Hay que moverla antes a los
pines 21 y 22 del ESP32. Si no lo has hecho, dirá "no responde en 0x68" y es
lo esperado.

```powershell
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test\target\test_imu
```

`s` para detectarla, `v` para la verificación guiada de signos.

- `SIGNO_ROLL` correcto / hay que invertirlo: ______
- `SIGNO_PITCH` correcto / hay que invertirlo: ______

## Paso 3 — CAN sin par · MKR + shield

> Antes: **desconecta el ESP32 del bus CAN.**

```powershell
arduino-cli upload -p COMx --fqbn arduino:samd:mkrwifi1010 test\target\test_can_mkr
```

1. `e` — barrido de IDs.
2. `d` — prueba del retardo de 250 µs.

| ID | ¿Responde? | Anomalías |
|---|---|---|
| 0x141 | | |
| 0x142 | | |
| 0x143 | | |
| 0x144 | | |

Resultado de la prueba `d`: pierde tramas sin retardo ☐ · con retardo ☐

> Si pierde sin el retardo y no pierde con él, es el fallo conocido del motor
> FL y **el motor está bien**.

## Paso 4 — CAN con movimiento · robot SOBRE TACOS

Mismo sketch, comando `m`. Un ID cada vez, 5 °/s durante 2 s, para solo.

| ID | Rueda que se mueve | Sentido |
|---|---|---|
| 0x141 | | |
| 0x142 | | |
| 0x143 | | |
| 0x144 | | |

## Paso 5 — Steppers · ESP32 · robot SOBRE TACOS

```powershell
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test\target\test_steppers_all
```

`a` para armar → `1`..`4` para elegir motor → `+` y `-` para moverlo.

Comprueba también que **los otros tres brazos no se mueven**.

| Motor | Brazo que mueve | ¿Sentido correcto? |
|---|---|---|
| FR | | |
| FL | | |
| RR | | |
| RL | | |

---

## Mapa de conexiones

```
            [Mando Xbox por Bluetooth]        (rama standalone)
                        |
                        v
   +--------------------------------------------------+
   |              ESP32 DevKit V1                      |
   +--------------------------------------------------+
     |                    |                     |
     | CAN                | GPIO x12            | I2C (21 SDA, 22 SCL)
     | GPIO 5 y 35        | step/dir/ena        |
     v                    v                     +------------------+
  [SN65HVD230]        [4x DM542]                |                  |
   transceptor         drivers                  v                  v
     |                    |                [TCA9548A 0x70]    [MPU9250 0x68]
     v                    v                     |               IMU (directa,
  [4x RMD-X8]        [4x steppers]              |                sin pasar por
   traccion           elevacion                 |                el mux)
   FL 0x141                                     +-- canal 0 -> AS5600 0x36  FR
   FR 0x142                                     +-- canal 1 -> AS5600 0x36  FL
   RR 0x143                                     +-- canal 2 -> AS5600 0x36  RR
   RL 0x144                                     +-- canal 3 -> AS5600 0x36  RL
```

El multiplexor existe **solo** porque los cuatro AS5600 comparten la misma
dirección fija (`0x36`) y no pueden convivir en un bus I2C. La IMU está en
`0x68`, no choca con nadie, y por eso va directa al bus.

### Pines de los steppers

Orden `{FR, FL, RR, RL}` en todos los vectores del proyecto.

| | FR | FL | RR | RL |
|---|---|---|---|---|
| STEP | 25 | 26 | 27 | 32 |
| DIR | 33 | 13 | 14 | 15 |
| ENA | 4 | 16 | 17 | 2 |

### Lo que todavía no existe

- **SN65HVD230**: sin comprar. Hasta que llegue, el ESP32 **no tiene bus CAN**
  y la tracción se prueba desde el MKR (paso 3 y 4).
- **IMU**: sigue cableada al MKR. Son tres hilos a los pines 21 y 22.

---

## Al terminar

Vuelca las tablas a los comentarios de las issues correspondientes:
encoders y offsets a la #1 y #5, tracción a la #2, steppers a la #3,
IMU a la #4.

Si actualizas `OFFSETS_ENCODER` en `config.h`, recuerda que hay que tocarlo en
**cuatro sitios**: `starcrawler_esp32`, `starcrawler_esp32_basico`,
`starcrawler_esp32_standalone` y `starcrawler_esp32_ros2`. Es lo que
describe la issue #10.
