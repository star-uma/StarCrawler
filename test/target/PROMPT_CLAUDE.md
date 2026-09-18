# Contexto para Claude

Pega el bloque de abajo al abrir una sesión nueva de Claude en este repo, o
simplemente dile: *"lee `test/target/PROMPT_CLAUDE.md` y sigue desde ahí"*.

Sirve para retomar el trabajo en otro PC sin arrastrar la conversación
anterior.

---

```
Vas a ayudarme con la puesta en marcha de StarCrawler, un robot de orugas
con cuatro brazos articulados. Estoy en el repo star-uma/StarCrawler, rama
feature/test-target-bringup.

## Estado real del proyecto

NADA está verificado en hardware todavía. Los tests de lógica pura pasan en
el PC (66/66) y los firmwares compilan, pero ni un encoder, ni un motor, ni
un ID de CAN se han comprobado con el robot delante. Ese es justo el trabajo
que estamos haciendo.

Antes de afirmar que algo funciona, comprueba si se ha verificado de verdad
o solo compila. No des por bueno lo que digan los docs sin contrastarlo con
el código.

## Arquitectura

Un solo ESP32 DevKit V1 lo controla todo (antes eran dos micros: un Arduino
MKR + un ESP32). Tres buses:

- CAN a 1 Mbps -> 4 motores RMD-X8 de tracción. IDs: FL 0x141, FR 0x142,
  RR 0x143, RL 0x144.
- 12 GPIO -> 4 drivers DM542 -> motores paso a paso de elevación.
  STEP {25,26,27,32}, DIR {33,13,14,15}, ENA {4,16,17,2}.
- I2C (SDA 21, SCL 22) -> multiplexor TCA9548A (0x70) con 4 encoders AS5600
  (0x36) + la IMU MPU9250 (0x68) directa al bus, SIN pasar por el mux.

El multiplexor existe solo porque los cuatro AS5600 comparten la misma
dirección fija. La IMU no choca con nadie, por eso va directa.

Orden en todos los vectores del proyecto: {FR, FL, RR, RL}.

## Lo que hay y lo que falta

Tenemos: ESP32, robot con encoders y motores cableados, Arduino MKR con su
shield CAN (funciona y ya ha hablado con estos motores a 1 Mbps), mini PC
para el futuro.

NO tenemos: transceptor CAN para el ESP32 (sin él, el ESP32 no puede hablar
con los motores de tracción), LiDAR. La IMU sigue cableada al MKR y habría
que moverla a los pines 21/22 del ESP32.

Por eso el diagnóstico de los motores de tracción se hace sobre el MKR: es
hardware ya verificado, y para aislar fallos conviene no meter variables
nuevas.

## Decisiones ya tomadas

- Rama de trabajo actual: feature/standalone-sin-pc (mando Xbox por
  Bluetooth directo al ESP32, sin PC). La rama ros2 es el destino a medio
  plazo, aparcada por ahora.
- Para el CAN del ESP32: TWAI + transceptor SN65HVD230 (~3 EUR), no un
  MCP2515. El backend TWAI ya está escrito en can_bus.cpp; solo falta el
  chip. Razonamiento completo en la issue #7.
- El firmware se queda en Arduino core, no se migra a ESP-IDF por ahora.
  FreeRTOS ya está debajo del core de Arduino, así que no hace falta
  cambiar de framework para tener tareas.

## Lo que toca hacer

Sigue test/target/GUIA_SESION.md. Son cinco pruebas, de menos a más riesgo:
encoders, IMU, CAN sin par, CAN con movimiento, steppers. Cada sketch tiene
un menú por puerto serie; escribe 'h' para la ayuda.

Tres reglas que no se saltan:
1. Robot sobre tacos antes de cualquier cosa que se mueva.
2. Nunca dos placas mandando en el bus CAN a la vez.
3. Un "no responde" es un resultado, se apunta igual.

## Trampa conocida

config.h define CAN_INTER_FRAME_US 250 con el comentario "fix del fallo
FL/0x141 del original". Hay un fallo conocido y ya corregido que afectaba al
motor FL cuando las tramas se mandaban demasiado seguidas. Si un motor hace
cosas raras, comprueba esto antes de darlo por averiado: el sketch
test_can_mkr tiene el comando 'd' para probarlo en los dos sentidos.

## Qué hacer con los resultados

Las tablas de GUIA_SESION.md se vuelcan a los comentarios de las issues:
encoders y offsets a la #1 y #5, tracción a la #2, steppers a la #3, IMU a
la #4. Hay 18 issues abiertas con el trabajo pendiente y el razonamiento de
cada decisión.

Si actualizo OFFSETS_ENCODER en config.h, recuérdame que hay que cambiarlo
en las CUATRO variantes de firmware (esp32, esp32_basico, esp32_standalone,
esp32_ros2): están duplicadas byte a byte, es la issue #10.

## Cómo quiero que trabajes

- Comentarios de código escuetos: el razonamiento va a los .md y a las
  issues, no al código.
- No commitees ni subas nada por iniciativa propia; déjalo en el árbol y yo
  decido.
- Si algo no lo has verificado, dilo. No hagas pasar por comprobado lo que
  solo compila.
- Español.
```

---

## Notas sobre este fichero

Está escrito a fecha de la primera sesión de puesta en marcha. Según vayan
cerrándose issues y verificándose cosas, **conviene actualizarlo**: sobre todo
el apartado "Estado real del proyecto", que es el que más rápido se queda
viejo y el que más daño hace si miente.
