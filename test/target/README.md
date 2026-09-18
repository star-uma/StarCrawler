# Pruebas con el robot delante (`test/target/`)

Estos cuatro programas sirven para **probar cada componente del robot por
separado**, antes de juntarlo todo. Están pensados para que los pueda usar
cualquiera del equipo, sin haber tocado el firmware antes.

La idea es simple: si algo falla, quieres saber **qué** falla. Con el robot
entero funcionando a la vez es imposible: si una oruga se mueve raro, puede
ser el motor, el encoder, el driver, el cable o el programa. Estos sketches
prueban **una cosa cada vez**.

---

## Antes de nada: las tres reglas

1. **El orden importa.** Van de menos a más riesgo. No te saltes pasos: la
   prueba 3 da por hecho que ya has hecho la 1.
2. **Robot sobre tacos** (con las orugas al aire, sin tocar el suelo) en
   cuanto algo se mueva. Pruebas 2 y 3.
3. **Nunca dos placas mandando en el bus CAN a la vez.** Si usas el MKR para
   la prueba 2, el ESP32 no puede estar transmitiendo por CAN, y al revés.

---

## Las cuatro pruebas

| # | Sketch | Placa | ¿Mueve motores? | Qué averigua |
|---|---|---|---|---|
| 1 | `test_encoders` | ESP32 | **No** | Que los 4 sensores de ángulo responden, cuál es cuál, y los offsets de calibración |
| 2 | `test_can_mkr` | MKR + shield CAN | Sí (opcional) | Qué ID tiene cada motor de tracción y cuál se porta mal |
| 3 | `test_steppers` | ESP32 | **Sí** | Que cada motor de elevación mueve el brazo que le toca, y en qué sentido |
| 4 | `test_imu` | ESP32 | **No** | Que el sensor de inclinación responde y que los signos son correctos |

### Orden recomendado

```
1. test_encoders    <- empieza aquí. Riesgo cero.
2. test_imu         <- riesgo cero también. Puedes hacerla cuando quieras.
3. test_can_mkr     <- primero el escaneo (no mueve), luego el movimiento
4. test_steppers    <- la que más cuidado requiere
```

---

## Preparar el PC (solo la primera vez)

Si el PC está limpio, ejecuta esto desde la carpeta del repo:

```powershell
.\test\target\instalar_entorno.ps1
```

Instala `arduino-cli`, el soporte de placa del ESP32 y del Arduino MKR, y la
librería CAN. Al terminar compila los cuatro sketches para comprobar que todo
ha quedado bien, y te dice cuál es el siguiente paso.

Tarda un rato la primera vez: el soporte del ESP32 son varios cientos de MB.
Se puede volver a ejecutar sin problema, se salta lo que ya esté instalado.

> El README principal del repo instala el soporte del Arduino MKR pero **no**
> el del ESP32, porque es anterior a la arquitectura nueva. Por eso hace falta
> este script: con el README principal solo se puede compilar la prueba 2.

Si Windows se queja de permisos para ejecutar scripts:

```powershell
powershell -ExecutionPolicy Bypass -File .\test\target\instalar_entorno.ps1
```

### Si la placa no aparece

Conecta la placa por USB y mira qué puerto le ha tocado:

```powershell
arduino-cli board list
```

Si no sale nada, casi siempre falta el driver USB:

| Placa | Driver |
|---|---|
| ESP32 DevKit V1 | CP2102 (Silicon Labs) o CH340 (WCH), según el chip que lleve |
| Arduino MKR WiFi 1010 | No necesita driver en Windows 10/11 |

---

## Cómo compilar y subir

Necesitas `arduino-cli` instalado (lo deja el script de arriba).

### Pruebas 1, 3 y 4 — ESP32

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 test/target/test_encoders
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test/target/test_encoders
```

Cambia `test_encoders` por `test_steppers` o `test_imu` según la que quieras,
y `COMx` por tu puerto (mira cuál es con `arduino-cli board list`).

### Prueba 2 — Arduino MKR WiFi 1010

```powershell
arduino-cli lib install "CAN"
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 test/target/test_can_mkr
arduino-cli upload -p COMx --fqbn arduino:samd:mkrwifi1010 test/target/test_can_mkr
```

### Y luego

Abre el **Monitor Serie a 115200 baudios**. Cada sketch te explica al arrancar
qué hace y qué comandos tiene. Escribe `h` y pulsa Enter para ver la ayuda en
cualquier momento.

> Solo un programa puede tener el puerto serie abierto a la vez. Si el monitor
> no se abre, cierra el que tengas en otra ventana.

---

## Qué tienes que apuntar

El objetivo de estas pruebas no es que "salga bien": es **salir con datos**.
Apunta los resultados, que luego hay que meterlos en `config.h`.

**De la prueba 1 (encoders):**

| Canal | Oruga que se mueve de verdad | Offset medido |
|---|---|---|
| 0 | | |
| 1 | | |
| 2 | | |
| 3 | | |

**De la prueba 2 (tracción):**

| ID | ¿Responde? | Rueda que se mueve | Sentido | Anomalías |
|---|---|---|---|---|
| 0x141 | | | | |
| 0x142 | | | | |
| 0x143 | | | | |
| 0x144 | | | | |

**De la prueba 3 (steppers):**

| Motor | Brazo que mueve | ¿El sentido es el esperado? |
|---|---|---|
| FR | | |
| FL | | |
| RR | | |
| RL | | |

**De la prueba 4 (IMU):** si `SIGNO_ROLL` y `SIGNO_PITCH` hay que invertirlos
o no.

---

## Preguntas frecuentes

**No se abre el Monitor Serie / no sale nada.**
Comprueba que la velocidad sea 115200. Si sigue en blanco, pulsa el botón de
reset de la placa: los sketches escriben la presentación solo al arrancar.

**Sale "NO RESPONDE" en todos los encoders.**
Casi siempre es el multiplexor: sin él no se lee ninguno. Mira que esté
alimentado a 3.3 V y que los pines A0/A1/A2 estén a GND.

**Ningún motor de tracción responde al escaneo.**
Antes de sospechar de los motores: que estén alimentados, que las resistencias
de terminación de 120 Ω estén **solo en los dos extremos** del bus, y que
CAN-H y CAN-L no estén cruzados.

**Un motor de tracción hace cosas raras.**
Usa el comando `d` de la prueba 2. Hay un fallo conocido del motor FL (`0x141`)
que aparece cuando las tramas se mandan demasiado seguidas, y que se arregla
con 250 µs de separación. Puede que el motor esté perfectamente.

**El brazo no se mueve pero el motor suena.**
El motor está perdiendo pasos o está desacoplado del brazo. Baja la velocidad
(sube `SEMIPERIODO_STEP_US` en el sketch) y mira el acoplamiento mecánico.

**Un stepper gira al revés.**
Es normal y es justo lo que la prueba busca. El sketch te dice qué valor hay
que invertir en `config.h`.

---

## Una nota sobre la configuración

Cada sketch tiene sus pines y direcciones **copiados** de
`firmware/starcrawler_esp32/config.h`, en un bloque marcado al principio del
archivo. Es a propósito: así cada prueba se compila sola, sin arrastrar el
firmware entero.

La contrapartida es que **si cambias un pin en `config.h`, hay que cambiarlo
también aquí**. Están todos juntos al principio de cada `.ino` para que sea
fácil de ver.
