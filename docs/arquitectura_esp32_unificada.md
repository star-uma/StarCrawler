# Arquitectura unificada: un solo ESP32 para todo StarCrawler

**Veredicto: SÍ es viable.** Un único ESP32 DevKit V1 + un módulo MCP2515 puede
asumir todo lo que antes hacían el Arduino MKR WiFi 1010 y el ESP32 juntos.
Este documento justifica la decisión, describe el nuevo firmware
(`firmware/starcrawler_esp32/`) y lista lo que hay que verificar con el robot delante.

## 1. Por qué cabe todo en un ESP32

| Recurso | Necesario | Disponible en ESP32 | Margen |
|---|---|---|---|
| GPIO de salida | 12 steppers (4×STEP/DIR/ENA) + 2 I2C + 4 SPI | 19 seguros | 1 libre + 3 input-only |
| CPU | Lazo 100 Hz + ISR steppers 833 Hz + WiFi | 2 núcleos a 240 MHz (WiFi va en el core 0, el sketch en el core 1) | enorme |
| CAN 1 Mbps | 200 tramas/s TX (4 motores × 50 Hz) | ~7800 tramas/s de capacidad de bus | ×39 |
| I2C 400 kHz | 4×AS5600 + TCA9548A + MPU9250 ≈ 2 ms/ciclo | 10 ms de ciclo | ×5 |
| WiFi/UDP | RX 50 Hz + TX telemetría 10 Hz | nativo | sobrado |

El MKR original solo aportaba: WiFi (el ESP32 ya lo tiene), el bus CAN (lo cubre
el MCP2515 por SPI, o el TWAI interno) y la IMU (se recablea al I2C del ESP32).

## 2. ⚠ Lo que hay que mirar del módulo MCP2515 antes de conectar

1. **Cristal de 16 MHz obligatorio.** Con el cristal de 8 MHz que llevan muchos
   módulos azules baratos **no se puede generar 1 Mbps** (los RMD-X8 van a
   1 Mbps). Mira el componente metálico del módulo: debe poner `16.000`.
2. **Tensiones.** El módulo típico (MCP2515 + transceptor TJA1050) necesita 5 V
   para el TJA1050, pero entonces el SPI habla a 5 V y el ESP32 es de 3.3 V.
   Opciones, de mejor a peor:
   - **Reutilizar la Shield CAN del MKR**: es un MCP2515 con cristal de 16 MHz y
     lógica a 3.3 V. Se cablea por SPI al ESP32 y ya funcionaba a 1 Mbps con los
     RMD. Es la opción con menos riesgo y coste cero.
   - Módulo MCP2515 modificado: cortar la pista de VCC del TJA1050 y alimentar
     MCP2515 a 3.3 V y TJA1050 a 5 V (mod muy documentado en internet).
   - Level shifter en MISO como mínimo (MOSI/SCK/CS suelen tolerar 3.3 V de entrada).
3. **Alternativa recomendada a futuro (backend ya implementado):** el ESP32 lleva
   controlador CAN interno (**TWAI**). Solo necesita un transceptor de 3.3 V
   (SN65HVD230, ~2 €). Libera el SPI y elimina los dos problemas anteriores.
   Se activa compilando con `-DCAN_BACKEND=2`:
   ```powershell
   arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --build-property "compiler.cpp.extra_flags=-DCAN_BACKEND=2" firmware/starcrawler_esp32
   ```

## 3. Nueva arquitectura

```
[PC — StarCrawlerXbox.py]  (SIN CAMBIOS)
        |  UDP 18 bytes @ 50 Hz  ->  192.168.10.101:8885
        |  <- telemetría 18 bytes @ 10 Hz al puerto 8886
        ▼
[ESP32 DevKit V1]  (único microcontrolador)
   ├── SPI -> MCP2515 -> CAN 1 Mbps -> RMD-X8: FL 0x141, FR 0x142, RR 0x143, RL 0x144
   ├── GPIO x12 -> 4x DM542 (STEP/DIR/ENA) -> steppers de elevación
   └── I2C (21/22) -> TCA9548A (0x70) -> 4x AS5600 (canales 0-3)
                   -> MPU9250 (0x68)   <- ¡recablear desde el MKR!
```

El ESP32 toma la IP estática que tenía el MKR (`192.168.10.101`), por lo que
**el script del PC no cambia en absoluto**. El MKR debe quedar fuera de la red.

### Cambios de cableado respecto al robot del TFG

| Qué | Antes | Ahora |
|---|---|---|
| IMU MPU9250 | I2C del MKR (pines 11/12) | I2C del ESP32 (21/22), junto al TCA9548A |
| Bus CAN | Shield CAN del MKR | MCP2515 por SPI del ESP32 (o la misma shield recableada) |
| Steppers/encoders | ESP32 (pines del TFG) | ESP32 con **mapa de pines nuevo** (ver tabla) |
| Arduino MKR | Tracción + puente | **Se elimina** |

### Mapa de pines del ESP32 (¡cambia respecto al TFG!)

Los pines originales del ESP32 (18, 19, 5…) chocaban con el SPI que ahora
necesita el MCP2515, así que el mapa es nuevo. Todo está en `config.h`.

| Función | Pines {FR, FL, RR, RL} |
|---|---|
| STEP | 25, 26, 27, 32 |
| DIR | 33, 13, 14, 15 |
| ENA | 4, 16, 17, 2 |
| SPI MCP2515 | SCK 18, MISO 19, MOSI 23, CS 5, INT 35 (sin usar, polling) |
| I2C | SDA 21, SCL 22 |
| TWAI (si backend 2) | TX 5, RX 35 |

Notas: GPIO 15 y 2 son pines de strapping; con los optoacopladores del DM542
como carga no interfieren en el arranque, pero si algún día el ESP32 no
arranca con los drivers conectados, ese es el primer sitio donde mirar.
GPIO 0 y 12 se han dejado libres a propósito (son los strapping conflictivos).

## 3b. Dos variantes de firmware

| | `starcrawler_esp32` (completa) | `starcrawler_esp32_basico` (sin IMU) |
|---|---|---|
| Modos | 1–5 | 1–4 (el 5 hace parada segura) |
| IMU MPU9250 | Sí (nivelado automático) | No — ni cableado ni código |
| Compensación de tracción | Sí | Sí |
| Resto | idéntico | idéntico |

La variante básica es para usar el robot **sin recablear la IMU**: tracción +
elevación y nada más. Los módulos comunes (`can_bus`, `steppers`) son copias
idénticas; `control_core` es el mismo sin las funciones de nivelado/actitud.

### Compensación de tracción (descubierta en Control_MKR_Completo.slx)

En el sistema original, durante los modos de elevación el MKR **no soltaba los
RMD**: los giraba para que la banda de la oruga no arrastrase mientras el brazo
bascula. Regla extraída de los Stateflow charts del modelo (4 casos por oruga):

- Oruga **subiendo** (ángulo del encoder disminuye) → RMD a **+5 °/s** («s.h»)
- Oruga **bajando** (ángulo aumenta) → RMD a **−5 °/s** («s.ah»)
- Oruga parada → velocidad 0 (el motor queda frenado, no libre)

El modelo usaba 5 °/s constante con ángulo <180° y una velocidad *variable*
con ángulo >180° cuya fórmula está en subsistemas gráficos del `.slx` (no
extraíble como texto); ambos firmwares usan constante en todo el rango como
aproximación (`COMPENSACION_DPS` en `config.h`, desactivable con
`COMPENSACION_TRACCION 0`). El signo eléctrico por motor
(`TABLA_SIGNO_COMPENSACION`, lado izquierdo invertido como en tracción) hay
que **verificarlo en hardware** con el robot sobre tacos.

## 4. Qué hace el firmware (y qué corrige respecto al original)

Los 5 modos del TFG están implementados: tracción (1), posición absoluta (2),
incremental ×4 (3), incremental ×2 (4) y nivelado automático (5) — el algoritmo
del nivelado es el puerto **literal** del Chart de Simulink (Código 7.4, límite 2°).

Correcciones sobre el firmware ESP32 original (detalladas en la revisión):

1. **Objetivos solo con paquete fresco** — el original copiaba un array sin
   inicializar cuando no llegaba paquete en el ciclo (consignas basura).
2. **Watchdog de 500 ms** — el original no tenía: si se caía el enlace en modo 3
   con una oruga girando, seguía girando indefinidamente.
3. **Lectura atómica del AS5600** — el original leía byte alto y bajo en dos
   transacciones y podía mezclar dos muestras (saltos de hasta ±22°).
4. **Encoder averiado ⇒ motor parado en modo 2** — el original seguía moviendo
   el motor con el último ángulo congelado (runaway hasta el tope mecánico).
5. **Histéresis en el control de posición** (arranque >1°, parada <0.5°) — el
   umbral único de 1° del original podía oscilar alrededor de la consigna.
6. **Límites software de recorrido** (85°–275°, configurable) cuando el encoder
   es válido.
7. **ISR sin `digitalRead/Write`** — escritura directa de registros GPIO, segura
   en IRAM y sin depender de la velocidad de la API de Arduino.
8. **Serial a 115200 y sin prints por paquete** — el original imprimía el modo
   50 veces/s a 9600 baudios (1000 B/s contra 960 B/s de capacidad: el buffer
   acababa bloqueando el lazo).
9. **Variables compartidas con la ISR declaradas `volatile`**.
10. **Vaciado del buffer UDP** quedándose con el último paquete (el original
    procesaba uno por ciclo y podía acumular retraso).

Decisión heredada a revisar: al parar un stepper se deshabilita el driver
(`PARADA_LIBERA_DRIVER 1`, como el original — la reductora 1:80 retiene). Si se
prefiere par de retención, ponerlo a 0 en `config.h`.

### Telemetría nueva (robot → PC, puerto 8886, 10 Hz)

`[0]=2, [1..4]=ángulos×100 {FR,FL,RR,RL}, [5]=roll×100, [6]=pitch×100, [7]=modo, [8]=bits de error`

Bits de error: 0–3 encoder FR/FL/RR/RL, 4 IMU, 5 CAN, 6 watchdog activo.

## 5. Estrategia de validación

| Nivel | Qué | Cómo | Hardware |
|---|---|---|---|
| 1. Unitarios | Toda la lógica pura (`control_core`) | `test/host/run_tests.ps1` (g++) | No |
| 2. Compilación | Firmware con ambos backends + sketches | `arduino-cli compile ...` | No |
| 3. Puesta en marcha | Cada subsistema por separado | `test/target/test_{can,encoders,imu,steppers}` | Sí |
| 4. HIL | Sistema completo por UDP real | `pytest test/hil -v -s` | Sí |

Estado actual: **niveles 1 y 2 pasados** (66/66 tests, compilación limpia).
Los niveles 3 y 4 requieren el robot.

### Orden recomendado de pruebas con el robot (de menos a más riesgo)

1. `test_encoders` — sin motores. Verifica mux + 4 AS5600 y los offsets.
2. `test_imu` — sin motores. **Verifica los signos de roll/pitch** (fig. 7.40:
   roll + = inclinado a la derecha, pitch + = hacia atrás). Si no coinciden,
   ajustar `SIGNO_ROLL`/`SIGNO_PITCH` en `config.h`. **No usar el modo 5 sin esto.**
3. `test_can` — solo envía "liberar" (sin par): motores alimentados pero quietos.
4. `test_steppers` — ¡robot sobre tacos! Verifica sentidos de giro de cada oruga.
5. Firmware completo + `pytest test/hil -v -s` — robot sobre tacos.
6. Modos 1→3→2→4→5 con el mando, en ese orden.

### Verificaciones pendientes que el código no puede resolver solo

- **Signos de la IMU** (punto 2 de arriba).
- **Modo 2 y objetivos simétricos**: el PC envía el mismo ángulo a las 4 orugas
  (p. ej. 225°), pero los comentarios del firmware original dicen que la pose
  "vertical arriba" es `{90,270,270,90}`. Si los encoders están montados en
  espejo la lectura ya es simétrica y no pasa nada; si no, el modo 2 del sistema
  original tenía este bug latente. Comprobar con `test_encoders` inclinando
  las orugas a mano.
- Los **offsets de encoder** `{-2,-5,16,-15}` son del robot en 2025; recalibrar.

## 6. Compilar y flashear el firmware

```powershell
arduino-cli lib install ACAN2515
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
```

El firmware compila tanto con el core esp32 2.x como con el 3.x (la API de
timers cambió entre ambos y hay un shim de compatibilidad en `steppers.cpp`).
