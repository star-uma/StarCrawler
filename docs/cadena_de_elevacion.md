# La cadena de elevación: qué es cada parámetro y por qué vale eso

Referencia de los motores paso a paso que articulan las orugas. No explica
cómo usar los sketches —eso está en [`test/target/`](../test/target/README.md)—
sino **de dónde sale cada número** del firmware, para poder cambiarlos
sabiendo qué se toca.

---

## La cadena completa

```
ESP32  --STEP/DIR/ENA-->  DM542  -->  57HS112  -->  reductora 1:80  -->  brazo
```

| Elemento | Qué es |
|---|---|
| **DM542** | Driver de micropaso (Walfront). Amplificador de corriente + controlador de movimiento |
| **57HS112-3004A08-D21** | Motor paso a paso NEMA 23, 4,2 A nominales |
| **Reductora 1:80** | El brazo gira a 1/80 de la velocidad del motor, con par de hasta 3 N·m |

El ESP32 no controla el motor: le manda **pulsos** al driver. Un pulso = un
paso. Todo lo que sigue es aritmética alrededor de eso.

---

## Los interruptores del DM542

El driver tiene un banco de interruptores que fija dos cosas independientes:

- **Corriente de fase** — cuánta corriente entrega al motor. Debe ajustarse al
  motor, no al capricho: el 57HS112 admite 4,2 A.
- **Micropasos** — en cuántos trozos parte cada paso completo del motor. La
  tabla serigrafiada en el propio driver dice qué combinación da qué valor,
  expresado en **pulsos por vuelta**.

> **Se leen al arrancar.** No son un latch que se guarde con un gesto ni se
> aplican en caliente: el driver los muestrea al recibir alimentación.
> Configúralos **con el driver sin tensión** y aliméntalo después. Cambiarlos
> encendido, en el mejor caso no hace nada.

También suele haber un interruptor de **reducción de corriente en reposo**
(half current), que baja la corriente cuando no llegan pulsos para que el
motor no se caliente parado.

### Qué está configurado aquí

El firmware da por supuestos **400 pulsos por vuelta**. El 57HS112 es un motor
de 1,8°, o sea 200 pasos completos por vuelta, así que 400 pulsos/vuelta
corresponde a **2 micropasos**.

Si alguien toca esos interruptores, **toda la aritmética de abajo cambia** y el
robot se moverá el doble o la mitad de lo que cree el firmware. Es de las cosas
que conviene comprobar antes de culpar al software.

---

## De pasos a grados

```
400 pulsos/vuelta × 80 de reductora = 32 000 pulsos por vuelta del brazo
32 000 / 360                        = 88,9 pulsos por grado de brazo
```

| Pulsos | Grados del brazo |
|---|---|
| 89 | 1° |
| 1 000 | 11° |
| 4 000 | 45° |
| 16 900 | 190°, el recorrido completo |

Esa última fila importa: **el recorrido mecánico es de 85° a 275°**, o sea 190
grados. Cualquier movimiento de más de ~16 900 pulsos lleva el brazo contra el
tope, donde se queda perdiendo pasos hasta que acabe la cuenta.

---

## El semiperiodo: la velocidad

El tren de pulsos se genera alternando el pin STEP. Un paso completo son **dos**
semiperiodos:

```
pulsos/s = 1 / (2 × semiperiodo)
```

| Semiperiodo | Pulsos/s | Velocidad del brazo |
|---|---|---|
| 4800 µs | 104 | 1,2 °/s |
| **1200 µs** | **417** | **4,7 °/s** |
| 2000 µs | 250 | 2,8 °/s |

### Por qué 1200 µs

Viene del TFG, y no es arbitrario: con el temporizador lanzando una
interrupción cada 1200 µs salen 833 Hz, que **está dentro de los límites de
frecuencia que admite el DM542** según la tabla del fabricante. Da una
velocidad de brazo de unos 5 °/s, que es cómoda de seguir con la vista y
suficiente para el robot.

Bajarlo acelera el brazo, pero el **par de un paso a paso cae con la
velocidad**. Hay un punto en el que el motor ya no puede con la inercia del
brazo y empieza a perder pasos sin avisar — el motor zumba y el brazo no se
mueve, o se mueve menos de lo que el firmware cree.

---

## La rampa de aceleración

El problema no es la velocidad de régimen: es **llegar a ella de golpe**.

Arrancar directamente a 417 pulsos/s con la inercia del brazo reflejada a
través de una reductora 1:80 hace que el motor pierda el sincronismo en los
primeros pulsos. La rampa arranca despacio y acelera:

| Constante | Valor | Qué significa |
|---|---|---|
| `SEMIPERIODO_ARRANQUE_US` | 4800 | Se arranca a 1,2 °/s |
| `SEMIPERIODO_STEP_US` | 1200 | Se acaba a 4,7 °/s |
| `RAMPA_DECREMENTO_TICKS` | 2 | Cuánto se acorta el semiperiodo por pulso |
| `RAMPA_ACTIVA` | 1 | Se puede desactivar para comparar |

Con esos valores la aceleración tarda unos **0,2 s**: 36 pulsos desde 96 ticks
hasta 24. La rampa se **rearma en cada cambio de sentido**, porque invertir el
giro es tan exigente como arrancar.

`test_steppers_all` y `test_steppers_one` traen el comando `r` para activarla y
desactivarla y comparar el mismo motor con y sin ella.

---

## El tick de la interrupción

Un solo temporizador tiene que generar cuatro trenes de pulsos que pueden ir a
velocidades distintas. La solución: el timer corre a un **tick fijo** y cada
motor cuenta ticks hasta su propio semiperiodo.

| Constante | Valor | Qué significa |
|---|---|---|
| `TICK_ISR_US` | 50 | La ISR corre a 20 kHz |

**El tick tiene que dividir exactamente a los dos semiperiodos.** Con 50 µs:
4800/50 = 96 ticks y 1200/50 = 24 ticks, ambos exactos. Si se cambia un
semiperiodo a un valor que no sea múltiplo de 50, la velocidad real no será la
que dice la constante.

Bajar el tick da más resolución pero sube la frecuencia de interrupción, que ya
está en 20 kHz.

---

## Los límites de recorrido

| Constante | Valor |
|---|---|
| `ANGULO_MIN_DEG` | 85° |
| `ANGULO_MAX_DEG` | 275° |

El rango mecánico del TFG es 90°–270° (vertical arriba y vertical abajo), y se
dejan **5° de margen** a cada lado. Son límites **software**: solo actúan si el
encoder de esa oruga responde. Sin encoder válido el firmware inhibe el lazo de
posición de ese brazo, porque mover a ciegas contra un tope es peor que no
moverse.

---

## El pin DIR

Dos cosas que no son evidentes y que ya han costado tiempo:

**Tiene que estar estable antes del pulso.** El DM542 muestrea DIR en el flanco
de STEP. Si se reescribe de forma asíncrona a los pulsos —por ejemplo en cada
ciclo de control a 100 Hz— puede caer justo en un flanco y el driver dará un
paso hacia el lado contrario. Por eso el firmware solo toca DIR al arrancar o
al invertir, con 10 µs de margen antes del primer pulso.

**Los pines `+` van a 3,3 V, no a 5 V.** Las entradas del DM542 son
optoacopladores. Con los `+` a 5 V y el ESP32 poniendo la señal en alto, quedan
1,7 V sobre el opto: no se apaga. El driver ve DIR permanentemente activado y
**el motor gira siempre hacia el mismo lado**, con independencia de lo que diga
el firmware. Con los `+` a 3,3 V el nivel alto deja 0 V y apaga de verdad.

A 3,3 V la corriente por el opto ronda los 8 mA, en el límite bajo de lo que
pide el driver. Funciona y es la solución estándar para controladores de 3,3 V,
pero si el driver dejara de responder del todo, ahí está la causa.

---

## La compensación de tracción

Mientras un brazo bascula, la banda de esa oruga arrastraría sobre el suelo si
el motor de tracción se quedara quieto. Para evitarlo, el RMD gira a la vez.

El comportamiento depende de qué polea toca el suelo:

| Ángulo | Qué apoya | Compensación |
|---|---|---|
| 90°–180° | polea **activa** | Girar en sentido opuesto al paso a paso, a **5 °/s constante** |
| 180°–270° | polea **pasiva** | **Variable con el ángulo** |

En el segundo caso, el desplazamiento horizontal del centro de la polea pasiva
es `x = d·(1 − cos α)`, y derivando respecto al tiempo sale la velocidad que
debe llevar el motor de tracción:

```
ω_servo [°/s] = (d · sin(α) · ω_stepper / R2) · (180/π)
```

| Símbolo | Qué es | Valor |
|---|---|---|
| `d` | distancia entre ejes de la oruga | **0,30445 m** (del CAD) |
| `R2` | radio de la rueda menor | pendiente |
| `α` | posición angular del centro de la rueda menor | variable |
| `ω_stepper` | velocidad del paso a paso | constante |

Está deducido en el capítulo 7 del TFG, ecuaciones 7.2 a 7.7, con la máquina de
estados en el código 7.1.

### Lo que falta antes de implementarla

El firmware usa hoy **5 °/s constante en todo el rango** como aproximación.
Para programar la fórmula real hay dos cosas sin cerrar:

- **El origen de α.** La memoria dice en un sitio que se mide «respecto a la
  horizontal» y en otro que la velocidad *disminuye* al acercarse a 270°. Con α
  desde la horizontal, `sin(α)` aumenta. Una de las dos cosas está mal.
- **Qué radio es `R2`.** Se define como «rueda menor» pero también se le llama
  «rueda motriz», que es la grande.

### Y una discrepancia con la tabla de signos

La memoria reparte la lógica **en diagonal** (FR igual que RL; FL y RR con los
estados intercambiados), lo que daría `{+1, −1, −1, +1}`. El firmware tiene
`{+1, −1, +1, −1}`, que es **izquierda/derecha**. No coinciden en RR ni en RL.

Puede que no sean comparables —el signo eléctrico del RMD y el espejado
geométrico de la oruga son cosas distintas— pero al verificar sobre tacos hay
que **mirar las cuatro orugas por separado**, no asumir simetría.

Todo el detalle está en la issue #6.

---

## Dónde vive cada constante

Todas en `config.h`, en las cuatro variantes de firmware:

```
firmware/starcrawler_esp32/config.h
firmware/starcrawler_esp32_basico/config.h
firmware/starcrawler_esp32_standalone/config.h   (rama standalone-sin-pc)
firmware/starcrawler_esp32_ros2/config.h         (rama ros2)
```

Están duplicadas byte a byte, así que **un cambio hay que replicarlo en las
cuatro**. Es lo que describe la issue #10.

Los sketches de `test/target/` tienen su propia copia de los valores al
principio de cada `.ino`, a propósito: así cada prueba compila sola sin
arrastrar el firmware.
