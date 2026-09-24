# Contexto para Claude — sesión en el PC personal

Sirve para retomar el trabajo en otro ordenador sin arrastrar la conversación
anterior. Abre Claude en este repo y dile:

> lee `README-CLAUDE.md` y sigue desde ahí

**Rama de trabajo: `feature/ros2`.** Es el tronco del proyecto. Las otras dos
ramas son para trabajar con el robot delante (`feature/test-target-bringup`,
con su propio `test/target/PROMPT_CLAUDE.md`) y para el modo sin PC
(`feature/standalone-sin-pc`).

---

## 1. Qué es el proyecto

StarCrawler es un robot de orugas con cuatro brazos articulados, proyecto de la
UMA y sucesor del robot Horu. Cada brazo lleva un motor de tracción (sin
escobillas, por CAN) y uno de elevación (paso a paso).

El objetivo de esta rama es que funcione **como Donatello**
(`star-uma/TFG_MARIA_JOSE`): ROS 2 corriendo en un mini PC a bordo, y el
microcontrolador reducido a capa de tiempo real.

Hardware, en orden `{FR, FL, RR, RL}` (es el orden de todos los vectores del
proyecto):

- **CAN a 1 Mbps** → 4 motores MyActuator RMD-X8-S2-V3, reducción 1:36.
  IDs: FL `0x141`, FR `0x142`, RR `0x143`, RL `0x144`.
- **12 GPIO** → 4 drivers DM542 → paso a paso con reductora 1:80 (elevación).
- **I2C** → multiplexor TCA9548A (`0x70`) con 4 encoders AS5600 (`0x36`),
  más la IMU MPU9250 (`0x68`) directa al bus, sin pasar por el mux.

El multiplexor existe solo porque los cuatro AS5600 comparten dirección fija.

---

## 2. Estado real — leer esto antes de afirmar que algo funciona

Mucho de este código **se escribió sin poder ejecutarlo**, en un ordenador donde
no se podía instalar ROS 2. Concretamente:

| Cosa | Estado |
|---|---|
| Los 9 paquetes de `ros2_ws/` | **`colcon build` limpio** (22-09-2026, WSL Ubuntu 22.04 + Humble, 30 s). Único arreglo necesario: `micro_ros_agent` no es clave de rosdep (`ecd4ced`) |
| `micro_ros_esp32_apps/starcrawler_app/` | **Compila** (22-09-2026) con `micro_ros_setup` humble → ESP-IDF **v4.1** (no 4.4): `starcrawler_app.bin` 459 KB, 0 errores/avisos. Arreglos: `idf_compat.h` y arrays fijos del `.msg` (`c16ad62`). Agente micro-ROS compilado. **Sin flashear ni probar** |
| Los 101 tests de lógica pura | **`colcon test`: 101/101** (22-09-2026). Los de lint (flake8/pep257/copyright) se declaran pero no se ejecutan |
| Robot simulado | **Levanta**: sim → odometría → GUI en `:8000`; `/odom` avanza al publicar `/cmd_vel` (22-09-2026) |
| ESP32 real con micro-ROS | **Funciona el camino completo** (22-09-2026, ESP32 DevKit V1 pelado por usbipd): agente ↔ nodo `starcrawler_esp32` ↔ odometría/GUI. Estado correcto sin hardware (`error_bits` 111). Telemetría a **49 Hz** (best-effort + transporte a 921600 desde la app, `fbf4e5a`/`add9179`). Reconexión automática si el agente se reinicia (ping, 5 s). **Ojo**: los 49 Hz se midieron con `ros2 topic hz`, que adapta el QoS; la GUI y la odometría se suscribían en fiable y con el ESP32 en best-effort no recibían nada. Arreglado en `db76c03`, **sin comprobar con el ESP32** |
| Mando DS4 → teleop | **Verificado en el banco de Windows** (22-09-2026): DS4 por Bluetooth → `tools/joy_bridge` (pygame→UDP) → `joy_udp_node` → teleop → twist_mux → sim. Stick izq. arriba = avance, stick der. derecha = `angular.z` negativo, X = preset. El mapeo de `ds4.yaml` estaba mal para el nodo `joy` (giro en el eje de L2, signos al revés): corregido (`7fd599e`) |
| Cualquier cosa con motores, encoders o CAN | Nada verificado en hardware |

Que un fichero exista y esté bien razonado no significa que compile. Lo normal
es que el primer `colcon build` saque errores de `package.xml`, `setup.py` y
dependencias. **Eso es el trabajo, no un imprevisto.**

No des por bueno lo que digan los docs sin contrastarlo con el código.

---

## 3. El entorno: WSL

Lo que **sí** se puede hacer en WSL:

- Compilar el workspace entero y correr los tests.
- Levantar el robot simulado y el grafo completo de ROS 2.
- La interfaz web: el nodo `starcrawler_gui` sirve en `http://localhost:8000`,
  y WSL2 reenvía localhost, así que se abre desde el navegador de Windows sin
  configurar nada.
- RViz, si hace falta: va por WSLg, que en Win11 viene de serie. Si sale en
  negro, `LIBGL_ALWAYS_SOFTWARE=1`.

Lo que **no** va a funcionar en WSL:

- **El mando no llega al WSL** (ni Bluetooth ni HID por usbipd). Solución que
  funciona: `tools/joy_bridge/joy_bridge.py` en Windows (Python 3.12, pygame) y
  lanzar con `joy_udp:=true`. El puente hay que arrancarlo desde una terminal
  de la sesión de escritorio, no desde un proceso sin sesión interactiva.
- **`rosdep install` sin terminal.** Llama a `sudo`, y si el usuario no tiene
  `NOPASSWD` se queda colgado esperando la contraseña sin decir nada. Lanzarlo
  desde una terminal interactiva o como root (`wsl -u root`).
- **GitHub por HTTPS falla con el protocolo v2 de git** en esta red ("expected
  flush after ref listing" y luego pide credenciales para repos públicos).
  Rompe `git submodule`, `vcs import` y el ExternalProject del agente. Arreglo:
  `git config --global protocol.version 0` (ya puesto en el WSL de Mario).
- **El ESP32 por serie funciona con `usbipd`** (probado 22-09-2026): `usbipd bind
  --busid X` una vez como administrador, `usbipd attach --wsl --busid X`, y aparece
  `/dev/ttyUSB0` (sin udev: no existe `/dev/starcrawler`, usar `port:=/dev/ttyUSB0`).
  Flashear con `ESPPORT=/dev/ttyUSB0 ros2 run micro_ros_setup flash_firmware.sh`
  desde `~/microros_ws`. Mientras está pasado al WSL, el COM desaparece de Windows.
- **El ESP32 por serie.** Requiere `usbipd-win`, y mientras el puerto está
  bindeado a WSL desaparece el COM de Windows. Para compilar y flashear
  firmware es más cómodo el Arduino IDE en Windows, sin WSL de por medio.

---

## 4. Lo que toca hacer, en orden

### Tarea 1 — elegir distribución de ROS 2 (issue #15) — HUMBLE EN USO, JAZZY EN PRUEBA (tarea 7)

Condiciona la versión de Ubuntu, así que va primero. Se montó Humble el
22-09-2026 y funciona; el 24-09 Mario pidió probar Jazzy antes de instalar el
mini PC. Ver la tarea 7.

- **Humble** → Ubuntu 22.04. Es la del laboratorio: Donatello y el entorno
  `uma_environment` van sobre Humble. Soporte hasta mayo de 2027.
- **Jazzy** → Ubuntu 24.04. Más soporte, pero separa del ecosistema del
  laboratorio.

Si el objetivo es "como Donatello", es Humble — y entonces hace falta un WSL de
22.04, que convive con el que ya haya sin tocar nada:

```bash
wsl --install -d Ubuntu-22.04
```

Comprobar dónde se está antes de instalar:

```bash
lsb_release -a
```

### Tarea 2 — instalar y compilar (issue #12) — HECHA 22-09-2026

```bash
./scripts/instalar_pc_abordo.sh humble
```

El script instala ROS 2, resuelve dependencias y configura el acceso al ESP32.
Se puede volver a ejecutar sin problema. Al final pregunta si habilitar el
arranque automático: **en WSL, no**.

Luego, desde `ros2_ws/`:

```bash
colcon build --symlink-install
```

Aquí es donde van a salir los fallos. Arreglarlos uno a uno hasta que
construya limpio. Los errores típicos del primer build son dependencias que
faltan en `package.xml`, `entry_points` mal puestos en `setup.py` y el paquete
de mensajes `starcrawler_msgs`, que es CMake y se construye distinto a los de
Python.

Un commit por causa arreglada, no todo en uno: si algo se rompe después, así se
localiza.

### Tarea 3 — pasar los tests — HECHA 22-09-2026

```bash
colcon test
```

```bash
colcon test-result --verbose
```

Son 101 tests de lógica pura, sin hardware ni ROS por debajo: conversión de
ángulos, protocolo serie, lógica del mando, odometría y el modelo del robot
simulado. Si alguno falla, mirar si el fallo está en el test o en el código —
ya pasó una vez que dos tests de odometría estaban mal y el código bien.

### Tarea 4 — levantar el robot simulado — HECHA 22-09-2026

```bash
ros2 launch starcrawler_bringup robot.launch.py sim:=true gui:=true teleop:=false
```

`odom` ya viene a `true` por defecto. Eso levanta la cadena entera sin
hardware: simulador → odometría → interfaz web. Se comprueba en
`http://localhost:8000` y con `ros2 topic echo /odom`.

Para que se mueva, sin mando:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

### Tarea 5 — compilar la app de micro-ROS — HECHA 22-09-2026

`micro_ros_esp32_apps/starcrawler_app/` es el ESP32 como nodo ROS 2 nativo. El
CAN, los paso a paso y los encoders se portaron a ESP-IDF **a ciegas**, así que
es lo que más probable es que esté roto.

Workspace ya montado en el WSL de Mario: `~/microros_ws` (micro_ros_setup +
agente en `install/`, firmware en `firmware/`, ESP-IDF v4.1 y toolchain en
`firmware/toolchain/`). Para recompilar tras tocar la app: copiar la carpeta a
`firmware/freertos_apps/apps/` y `ros2 run micro_ros_setup build_firmware.sh`.
Instrucciones completas en `micro_ros_esp32_apps/starcrawler_app/README.md`.

---

### Tarea 6 — vista 3D del robot moviéndose por el plano — ESCRITA 23-09-2026, SIN PROBAR CON ROS

Mario eligió **los dos caminos**, y están los dos:

- **RViz**: `plano.rviz`, con marco fijo `odom`, el rastro de `/odom` y la
  cámara siguiendo al robot. Lo usan `robot.launch.py` (`rviz:=true`) y
  `rviz.launch.py`. El `starcrawler.rviz` de antes fijaba `base_footprint`: el
  robot se quedaba en el centro y solo se movían los brazos. Sigue para
  `view_model.launch.py`.
- **Web 3D** en `http://localhost:8000/3d` (`starcrawler_gui`, three.js). El
  modelo no está escrito en la página: sale de `/robot_description`, traducido
  por `urdf_modelo.py`, así que dibuja lo mismo que RViz y con los mismos
  signos.

Qué está verificado y qué no:

- `urdf_modelo.py` y la página: 14 tests puros que pasan. El test que expande
  el xacro real **se salta sin `xacro`**; con ROS cargado tiene que correr.
- La página se ha visto en un navegador **con datos simulados**, sin ROS: el
  robot se dibuja, se desplaza, el rastro sale continuo y los signos de los
  brazos cuadran (positivo = levantado).
- **Sin probar**: el `gui_node` real con las tres suscripciones nuevas y
  `plano.rviz` abierto en RViz. Primera prueba:

```bash
ros2 launch starcrawler_bringup robot.launch.py sim:=true gui:=true rviz:=true teleop:=false
```

Con eso, publicar un `/cmd_vel` y ver que el robot avanza en las dos vistas a
la vez. Si RViz lo pinta bien y la web no, el fallo está en la web.

Arreglado de paso, porque rompía la vista con el ESP32 real: la GUI y la
odometría se suscribían a `starcrawler/state` en modo fiable, y desde
`fbf4e5a` el ESP32 publica en best-effort. No casaban y no les llegaba nada.
`ros2 topic hz` no lo delata porque adapta el QoS solo. Y la odometría ahora
deja de integrar si pasan 0,5 s sin estado.

### Tarea 7 — probar Jazzy en paralelo, sin tocar Humble (pedida por Mario, 24-09-2026)

**Por qué.** Humble tiene soporte hasta mayo de 2027; Jazzy, hasta mayo de
2029. Si el proyecto sigue después de 2027 (otro TFG que lo continúe), en
Humble se queda sin soporte. El mini PC del robot aún no está instalado, así
que ahora es cuando cambiar sale barato.

**No está decidido.** Mario quiere saber cuánto cuesta. Esta tarea es
probarlo y dejar apuntado qué ha pasado; la decisión es suya (issue #15).
Hasta entonces, Humble sigue siendo el entorno de referencia.

**Antes de empezar, cerrar lo pendiente en Humble**, para tener una base
limpia. Si no, un fallo en Jazzy no se sabrá si viene de Jazzy o del código
nuevo:

- La tarea 6: la vista 3D y `plano.rviz` con el robot simulado.
- Con el ESP32 pelado: que la GUI y la odometría **reciban** su estado (el
  arreglo de QoS de `db76c03`). Comprobarlo con `ros2 topic echo /odom` o
  viendo que la GUI marca enlace, **no** con `ros2 topic hz`, que adapta el QoS
  solo y fue justo lo que ocultó el fallo.

**Reglas**

- No tocar la distro `Ubuntu-22.04` de WSL ni su `~/microros_ws`. Jazzy va en
  una distro nueva, `Ubuntu-24.04`, y las dos conviven.
- El código tiene que seguir compilando en Humble. Lo que haga falta cambiar
  para Jazzy, que sirva en los dos; en la app del ESP32, distinguiendo por
  versión de ESP-IDF en `idf_compat.h`, que ya lo hace así.
- No flashear el robot: solo el ESP32 pelado del banco.
- No cambiar el `humble` por defecto de `instalar_pc_abordo.sh` ni los docs
  hasta que Mario decida.

**Pasos**

1. La distro nueva. Antes, `wsl -l -v`, por si ya hay una 24.04 con otras cosas.

```bash
wsl --install -d Ubuntu-24.04
```

2. Dentro, lo mismo que el lunes en Humble: `git config --global
   protocol.version 0` (la trampa de la red, ver sección 3), clonar
   `feature/ros2` y lanzar la instalación **desde una terminal interactiva**,
   porque `rosdep` pide el `sudo`:

```bash
./scripts/instalar_pc_abordo.sh jazzy
```

3. `colcon build --symlink-install` y `colcon test`. Lo esperado son **116
   tests** (los 101 de antes más 15 de la GUI) y **ninguno saltado**: con ROS
   cargado hay `xacro`, así que el test del URDF real corre. Avisos de
   setuptools por el Python 3.12 de Ubuntu 24.04 son normales; errores, no.
4. La tarea 6 otra vez, ahora en Jazzy: el launch con `sim:=true gui:=true
   rviz:=true teleop:=false` y las dos vistas.
5. micro-ROS: `micro_ros_setup` en su rama `jazzy`, en un `~/microros_ws` de la
   distro nueva, y `create_firmware_ws.sh freertos esp32`. **Apuntar qué versión
   de ESP-IDF trae**: con humble era la 4.1, y todo lo de abajo depende de eso.
   Copiar la app y compilar. Donde espero que salte algo:
   - `app.c` reutiliza `esp32_serial_open/close/write/read`, el transporte
     serie de `freertos_apps`, para subir el UART a 921600. Si en la rama
     jazzy se llaman distinto o cambian de firma, hay que adaptarlo. Es lo
     primero que hay que mirar.
   - Si trae ESP-IDF 5.x, el timer (`driver/timer.h`) y el I2C
     (`driver/i2c.h`) de `hw.c` son los drivers antiguos: deberían compilar
     con avisos de obsoleto. No pasarlos a `gptimer`/`i2c_master` en esta
     tarea, salvo que no compile.
6. Flashear el ESP32 pelado y repetir lo del lunes: sesión con el agente,
   `error_bits` 111, las tasas (~49 Hz), la reconexión al reiniciar el agente,
   y que la GUI y la odometría reciban el estado.
7. El mando por el puente UDP, igual que en Humble.

**Qué dejar apuntado aquí**, para que Mario decida: una tabla Humble frente a
Jazzy, paso por paso (funciona / con avisos / roto y qué se tocó), la versión de
ESP-IDF, y más o menos cuánto tiempo llevó. Y proponerle a Mario un resumen para
la issue #15.

## 5. Decisiones ya cerradas — no las vuelvas a abrir

Media sesión se puede ir en rediscutir cosas que ya están decididas. Estas lo
están:

- **Sin LiDAR ni mapeo.** Se quitaron de esta rama a propósito: el robot no
  lleva LiDAR y nav2 no pinta nada todavía. No lo vuelvas a meter.
- **CAN del ESP32: TWAI interno + transceptor SN65HVD230**, no un MCP2515. El
  backend TWAI ya está escrito en `can_bus.cpp`; falta el chip. El porqué está
  en la issue #7.
- **Gazebo, descartado por ahora.** No modela orugas de forma nativa, y el
  esfuerzo no compensa frente al simulador a nivel de tópicos que ya existe en
  `starcrawler_sim`.
- **Las mallas CAD del URDF están bloqueadas**: hacen falta FreeCAD y los
  ficheros del CAD. Mientras tanto el URDF va con primitivas.
- **Los firmwares de `firmware/` siguen en Arduino core**, no se migran a
  ESP-IDF. La app de `micro_ros_esp32_apps/` sí es ESP-IDF, porque es lo que
  pide micro-ROS.

Si crees de verdad que alguna hay que reabrirla, dilo y que lo decida Mario.
Por tu cuenta, no.

---

## 6. Lo que no hay que hacer

- **No commitear notas ni docs por iniciativa propia.** El razonamiento va a
  los `.md` que se pidan y a las issues, no a comentarios en el código.
- **Commits sencillos**: asunto y dos o tres líneas como mucho. **Sin línea de
  coautor.**
- No tocar `main`: está congelado y se arregla cuando se mergee.
- No dar por verificado lo que solo compila.

---

## 7. Mapa del repo

```
ros2_ws/src/
  starcrawler_msgs/         CrawlerCommand y RobotState (CMake)
  starcrawler_common/       conversión de ángulos, única fuente de verdad
  starcrawler_driver/       puente serie con el ESP32 (protocolo con CRC16)
  starcrawler_sim/          robot simulado a nivel de tópicos
  starcrawler_odometry/     odometría de orugas y TF odom->base
  starcrawler_teleop/       mando DS4 + twist_mux
  starcrawler_gui/          interfaz web en :8000
  starcrawler_description/  URDF y RViz
  starcrawler_bringup/      launch, systemd y udev

micro_ros_esp32_apps/starcrawler_app/   el ESP32 como nodo ROS 2 nativo
firmware/                               las variantes de Arduino
scripts/instalar_pc_abordo.sh           instalación del mini PC
docs/ros2.md                            arquitectura y puesta en marcha
docs/cadena_de_elevacion.md             de dónde sale cada constante
```

Hay **tres formas excluyentes** de que el grafo reciba estado, y el launch las
separa con argumentos: `starcrawler_driver` (serie, lo de ahora), el agente de
micro-ROS (`micro_ros:=true`, el destino) y `starcrawler_sim` (`sim:=true`, sin
hardware).

---

## 8. Issues abiertas que tocan esto

- **#12** — primer `colcon build` del workspace.
- **#15** — decidir Humble o Jazzy.
- **#21** — el bus CAN se cae con los cuatro motores (hardware, bloqueante).
- **#7** — transceptor CAN del ESP32.
- **#6** — la fórmula de compensación de tracción.

Todo en `github.com/star-uma/StarCrawler/issues`.
