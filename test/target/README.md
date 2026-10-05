# Pruebas con el robot delante (`test/target/`)

Estos programas sirven para **probar cada componente del robot por
separado**, antes de juntarlo todo. Están pensados para que los pueda usar
cualquiera del equipo, sin haber tocado el firmware antes.

La idea es simple: si algo falla, quieres saber **qué** falla. Con el robot
entero funcionando a la vez es imposible: si una oruga se mueve raro, puede
ser el motor, el encoder, el driver, el cable o el programa. Estos sketches
prueban **una cosa cada vez**.

---

## Antes de nada: las tres reglas

1. **El orden importa.** Van de menos a más riesgo. No te saltes pasos: la
   prueba 4 da por hecho que ya has hecho la 1.
2. **Robot sobre tacos** (con las orugas al aire, sin tocar el suelo) en
   cuanto algo se mueva. Pruebas 3 y 4.
3. **Nunca dos placas mandando en el bus CAN a la vez.** Si usas el MKR para
   la prueba 3, el ESP32 no puede estar transmitiendo por CAN, y al revés.

---

## Las pruebas

### Recorrido base

El orden de una puesta en marcha. Con esto cubres el robot entero.

| # | Sketch | Placa | ¿Mueve motores? | Qué averigua |
|---|---|---|---|---|
| 1 | `test_encoders` | ESP32 | **No** | Que los 4 sensores de ángulo responden, cuál es cuál, y los offsets de calibración |
| 2 | `test_imu` | ESP32 | **No** | Que el sensor de inclinación responde y que los signos son correctos |
| 3 | `test_can_mkr` | MKR + shield CAN | Sí (opcional) | Qué ID tiene cada motor de tracción y cuál se porta mal |
| 4 | `test_steppers_all` | ESP32 | **Sí** | Que cada motor de elevación mueve el brazo que le toca, y en qué sentido |

### Sketches de diagnóstico

No forman parte del recorrido; se usan cuando algo del recorrido base falla y
hay que averiguar por qué.

| Sketch | Placa | Para qué |
|---|---|---|
| `test_steppers_one` | ESP32 | Un único stepper, sin encoders ni I2C. Para comprobar que gira y que el sentido de DIR es el esperado, con el cableado mínimo |
| `test_par_stepper` | ESP32 | Por qué un stepper pierde pasos. Separa par de retención, arranque y velocidad, sin necesitar encoder |
| `test_motor_diag` | MKR + shield CAN | Fallos intermitentes de **un** motor de tracción: vuelca telemetría en CSV (velocidad real, corriente, temperatura) para verlo con datos y no a ojo |
| `test_can_esp32` | ESP32 | Validar el transceptor CAN del ESP32 por TWAI, antes de confiarle los motores |
| `test_i2c_escaner` | ESP32 | El bus I2C: pull-ups, líneas en corto y qué responde en cada dirección. Ver [encoders, paso a paso](#encoders-paso-a-paso) |
| `test_as5600_solo` | ESP32 | **Un** encoder sin multiplexor: imán, ganancia, ruido, una vuelta completa y sentido |
| `test_tca9548a` | ESP32 | El multiplexor solo, y luego con un encoder canal a canal |

### Orden recomendado

```
1. test_encoders      <- empieza aquí. Riesgo cero.
2. test_imu           <- riesgo cero también. Puedes hacerla cuando quieras.
3. test_can_mkr       <- primero el escaneo (no mueve), luego el movimiento
4. test_steppers_all  <- la que más cuidado requiere
```

Si en el paso 4 un motor pierde pasos, sigue con `test_par_stepper`. Si en el
3 un motor va raro de forma intermitente, con `test_motor_diag`.

---

## Encoders, paso a paso

`test_encoders` necesita que **todo** funcione a la vez: el bus, el
multiplexor y los cuatro sensores. Si falla, no dice qué pieza es. Para eso
están estos pasos, que van añadiendo piezas de una en una. Úsalos al montar
los encoders por primera vez, o cuando `test_encoders` falle.

| Paso | Sketch | Qué va conectado | Qué averigua |
|---|---|---|---|
| 1 | `test_i2c_escaner` | Lo que haya, aunque sea nada | Si el bus está sano y qué responde |
| 2 | `test_as5600_solo` | **Un** AS5600 directo al ESP32 | Si ese sensor y su imán van bien |
| 3 | `test_tca9548a` (`m`) | El multiplexor solo | Si el multiplexor funciona |
| 4 | `test_tca9548a` (`c`) | Multiplexor + **un** AS5600 | Si cada canal funciona |
| 5 | `test_encoders` | Todo | Que canal es cada oruga, sentido y offsets |

Ninguno mueve motores ni escribe en los sensores.

**La regla para localizar un fallo**: cambia una sola cosa cada vez. Un
encoder que va bien en el paso 2 y falla en el 4 apunta al canal o a su
cable; uno que falla ya en el paso 2 es el sensor, su imán o su cable.

### Cómo se conecta cada montaje

Todo a **3,3 V**, nunca a 5 V: el ESP32 no aguanta 5 V en sus pines.

**Paso 2: un AS5600 solo** (sin el multiplexor)

| ESP32 | AS5600 |
|---|---|
| 3V3 | VCC |
| GND | GND |
| GPIO21 | SDA |
| GPIO22 | SCL |
| GND | DIR |
| — | OUT y GPO, sin conectar |

**DIR va a GND, no al aire.** Al aire, el sentido de giro del sensor puede
cambiar solo. Y los cuatro sensores del robot, con DIR igual: el espejado de
FL y RR lo hace el software.

El imán tiene que ser de **magnetización diametral** (los polos a los lados,
no arriba y abajo), centrado sobre el chip y a unos 0,5–3 mm.

**Paso 3: el multiplexor solo**

| ESP32 | TCA9548A |
|---|---|
| 3V3 | VIN |
| GND | GND |
| GPIO21 | SDA |
| GPIO22 | SCL |
| GND | A0, A1 y A2 (dirección 0x70) |
| 3V3 | RST, si el módulo no lo trae ya a 3,3 V |

Con RST al aire, el chip se resetea solo de vez en cuando y los encoders
"desaparecen" a ratos. `test_tca9548a` lo detecta.

**Paso 4: el multiplexor con un encoder**

El montaje del paso 3, y el AS5600 en un canal:

| TCA9548A | AS5600 |
|---|---|
| SD0 | SDA |
| SC0 | SCL |

Y el AS5600 con VCC a 3V3, GND a GND y DIR a GND, como en el paso 2.
Después se pasa al canal 1, al 2 y al 3, y se repite: cada canal se prueba
con un sensor que ya sabes que funciona.

Cada canal necesita sus propias resistencias de **pull-up** (4,7 kΩ de SDx y
SCx a 3,3 V). Muchos módulos AS5600 las llevan. Si el tuyo no, el canal no
funciona aunque el multiplexor esté bien.

**Paso 5, el robot**: canal 0 = FR, 1 = FL, 2 = RR, 3 = RL. La IMU va
directa al bus principal, sin pasar por el multiplexor.

### Lo que tiene que salir

| Paso | Bien | Si no |
|---|---|---|
| 1 | SDA y SCL con pull-up; 0x70 (mux) y 0x68 (IMU); ningún 0x36 con el mux cerrado | Un 0x36 en el bus principal es un sensor conectado sin pasar por el mux |
| 2 | Imán `ok`, ruido de menos de 10 cuentas, las 64 zonas de la vuelta, ganancia estable | Imán débil o fuerte: distancia. Ganancia que baila en la vuelta: imán descentrado |
| 3 | Responde en 0x70, el registro guarda lo que se escribe, no se resetea | En 0x71–0x77: A0–A2 no están a GND. Se resetea: RST |
| 4 | El AS5600 aparece en un canal y solo en ese | Contesta pero falla al leer: pull-ups del canal |
| 5 | Los cuatro responden, cada brazo en su sentido (`u`), offsets entre −85° y +85° | Un offset fuera de ±85°: imán girado (ver el aviso del asistente) |

### El salto de 360°

El firmware calcula el ángulo como `crudo × 0,088 + offset`, **sin dar la
vuelta** a los 360°. Funciona mientras el salto del sensor (de 4095 a 0)
caiga fuera del recorrido del brazo, que va de 85° a 275°: o sea, con
offsets entre −85° y +85°. Si un imán se monta girado, el asistente de
offsets da un número más grande y el ángulo de esa oruga saltaría 360° en
mitad del movimiento. El asistente avisa; el arreglo de fondo es la #28.

Cada paso tiene su issue, colgando de la #1: #24 (bus), #25 (un encoder),
#26 (multiplexor solo) y #27 (multiplexor con un encoder).

---

## Preparar el PC (solo la primera vez)

Si el PC está limpio, ejecuta esto desde la carpeta del repo:

```powershell
.\test\target\instalar_entorno.ps1
```

Instala `arduino-cli`, el soporte de placa del ESP32 y del Arduino MKR, y la
librería CAN. Al terminar compila todos los sketches para comprobar que todo
ha quedado bien, y te dice cuál es el siguiente paso.

Tarda un rato la primera vez: el soporte del ESP32 son varios cientos de MB.
Se puede volver a ejecutar sin problema, se salta lo que ya esté instalado.

> El README principal del repo instala el soporte del Arduino MKR pero **no**
> el del ESP32, porque es anterior a la arquitectura nueva. Por eso hace falta
> este script: con el README principal solo se pueden compilar los del MKR.

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

### Sketches que van en el ESP32

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 test/target/test_encoders
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test/target/test_encoders
```

Cambia `test_encoders` por el sketch que quieras (`test_imu`,
`test_steppers_all`, `test_par_stepper`...),
y `COMx` por tu puerto (mira cuál es con `arduino-cli board list`).

### Sketches que van en el Arduino MKR

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

**De la prueba 3 (tracción):**

| ID | ¿Responde? | Rueda que se mueve | Sentido | Anomalías |
|---|---|---|---|---|
| 0x141 | | | | |
| 0x142 | | | | |
| 0x143 | | | | |
| 0x144 | | | | |

**De la prueba 4 (steppers):**

| Motor | Brazo que mueve | ¿El sentido es el esperado? |
|---|---|---|
| FR | | |
| FL | | |
| RR | | |
| RL | | |

**De la prueba 2 (IMU):** si `SIGNO_ROLL` y `SIGNO_PITCH` hay que invertirlos
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
Usa el comando `d` de la prueba 3. Hay un fallo conocido del motor FL (`0x141`)
que aparece cuando las tramas se mandan demasiado seguidas, y que se arregla
con 250 µs de separación. Puede que el motor esté perfectamente.

**El brazo no se mueve pero el motor suena.**
El motor está perdiendo pasos o está desacoplado del brazo. Usa `test_par_stepper`,
que separa las tres causas típicas. El firmware ya lleva rampa de aceleración
(`RAMPA_ACTIVA` en `config.h`): `test_steppers_all` tiene el comando `r` para
compararlo con y sin ella.

**El motor gira siempre hacia el mismo lado, con `+` y con `-`.**
Es un problema de cableado, no del sketch. Los pines `PUL+`, `DIR+` y `ENA+`
del DM542 tienen que ir a **3,3 V**, no a 5 V: las entradas son
optoacopladores, y con 5 V el nivel alto del ESP32 deja 1,7 V sobre el opto,
que no llega a apagarlo. El driver ve DIR siempre activado.
Para confirmarlo sin mover el motor, usa el comando `d` de
`test_steppers_one` y mide entre `DIR+` y `DIR-`: si en estado alto marca
~1,7 V en vez de ~0 V, es esto.

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
