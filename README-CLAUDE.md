# Contexto para Claude — sesión en el PC personal

Para retomar el trabajo sin arrastrar conversaciones anteriores. Abre Claude en
este repo y dile: *"lee `README-CLAUDE.md` y sigue desde ahí"*.

**Rama de trabajo: `feature/ros2`**, el tronco. Las otras son para probar con
el robot delante (`feature/test-target-bringup`, con su propio
`test/target/PROMPT_CLAUDE.md`) y para el modo sin PC
(`feature/standalone-sin-pc`).

---

## 1. El proyecto

Robot de orugas con cuatro brazos articulados, de la UMA, sucesor del Horu. El
objetivo de esta rama es que funcione **como Donatello**
(`star-uma/TFG_MARIA_JOSE`): ROS 2 en un mini PC a bordo y el ESP32 como capa
de tiempo real, hablando micro-ROS.

Hardware, siempre en orden `{FR, FL, RR, RL}`:

- **CAN a 1 Mbps** → 4 MyActuator RMD-X8-S2-V3 (1:36). IDs: FL `0x141`,
  FR `0x142`, RR `0x143`, RL `0x144`.
- **12 GPIO** → 4 drivers DM542 → paso a paso con reductora 1:80.
- **I2C** → TCA9548A (`0x70`) con 4 AS5600 (`0x36`, de ahí el mux), y la IMU
  MPU9250 (`0x68`) directa al bus.

---

## 2. Estado real — leer antes de afirmar que algo funciona

Mucho código se escribió sin poder ejecutarlo. **Lo que aquí no pone
"verificado" no lo está**, y no des por bueno lo que digan los docs sin
contrastarlo con el código.

| Cosa | Estado |
|---|---|
| Workspace `ros2_ws/` | Verificado: `colcon build` limpio y 101/101 tests (22-09, WSL Ubuntu 22.04 + Humble). Los tests de lint se declaran pero no corren |
| Robot simulado | Verificado: sim → odometría → GUI en `:8000` (22-09) |
| App de micro-ROS | Verificado con un ESP32 **sin nada conectado** (22-09): sesión con el agente, `error_bits` 111 (encoders + CAN + watchdog), ~49 Hz a 921600, reconexión en 5 s si el agente se reinicia. ESP-IDF **4.1** (la de `micro_ros_setup` humble), con `idf_compat.h` |
| GUI y odometría con el ESP32 | **Sin verificar.** Se suscribían en fiable y el ESP32 publica en best-effort: no recibían nada. Arreglado en `db76c03`. Los 49 Hz se midieron con `ros2 topic hz`, que adapta el QoS y no lo delató |
| Mando DS4 | Verificado en Windows por el puente UDP (22-09). El mapeo de `ds4.yaml` se corrigió en `7fd599e`. **Sin probar con el nodo `joy` real** del mini PC |
| Vistas 3D (tarea 6) | Escritas; tests y navegador con datos simulados. **Sin probar con ROS** |
| Motores, encoders, CAN | **Nada verificado en hardware** |

---

## 3. El entorno: WSL de Mario

- **La interfaz web** (`http://localhost:8000` y `/3d`) se abre desde el
  navegador de Windows: WSL2 reenvía localhost.
- **RViz** va por WSLg. Si sale en negro, `LIBGL_ALWAYS_SOFTWARE=1`.
- **El mando no llega al WSL.** Se usa `tools/joy_bridge/joy_bridge.py` en
  Windows (Python 3.12, pygame) y se lanza con `joy_udp:=true`. El puente se
  arranca desde una terminal del escritorio, no desde un proceso sin sesión.
- **El ESP32 por `usbipd`**: `usbipd bind --busid X` una vez como
  administrador, luego `usbipd attach --wsl --busid X`, y aparece
  `/dev/ttyUSB0` (no hay udev: usar `port:=/dev/ttyUSB0`). Mientras está en el
  WSL, el COM desaparece de Windows.
- **micro-ROS** está en `~/microros_ws`. Para recompilar tras tocar la app:
  copiarla a `firmware/freertos_apps/apps/` y `ros2 run micro_ros_setup
  build_firmware.sh`. Flashear con `ESPPORT=/dev/ttyUSB0 ros2 run
  micro_ros_setup flash_firmware.sh`.

Trampas ya encontradas:

- **`rosdep` se cuelga sin avisar** esperando la contraseña de `sudo`.
  Lanzarlo desde una terminal interactiva o con `wsl -u root`.
- **GitHub por HTTPS falla con el protocolo v2 de git** en esa red ("expected
  flush after ref listing"). Rompe `git submodule`, `vcs import` y la
  compilación del agente. Arreglo: `git config --global protocol.version 0`.

---

## 4. Lo que toca hacer, en orden

### Hecho

- **Tarea 1**, distribución: Humble en uso (22-09). Jazzy en prueba, ver la
  tarea 7.
- **Tareas 2 a 5** (22-09): instalar y compilar, los tests, el robot simulado y
  la app de micro-ROS. Ver la tabla de la sección 2.

### Tarea 6 — probar las vistas del plano con ROS

Hay dos, y las dos dibujan el mismo URDF:

- **RViz**: `plano.rviz`, marco fijo `odom`, rastro de `/odom` y la cámara
  siguiendo al robot. Lo usan `robot.launch.py` (`rviz:=true`) y
  `rviz.launch.py`. `starcrawler.rviz` fija `base_footprint` y queda para
  `view_model.launch.py`.
- **Web 3D** en `:8000/3d`. El modelo sale de `/robot_description`, traducido
  por `urdf_modelo.py`; la pose de `/odom` y los brazos de `/joint_states`.

```bash
ros2 launch starcrawler_bringup robot.launch.py sim:=true gui:=true rviz:=true teleop:=false
```

Publicar un `/cmd_vel` y ver que el robot avanza en las dos vistas a la vez.
Si RViz lo pinta bien y la web no, el fallo está en la web. `colcon test`
tiene que dar **116** (101 + 15 de la GUI) y **ninguno saltado**: con ROS
cargado hay `xacro` y corre el test del URDF real.

Después, con el ESP32 pelado: que la GUI y la odometría **reciban** su estado.
Comprobarlo con `ros2 topic echo /odom` o viendo que la GUI marca enlace,
**no** con `ros2 topic hz`.

### Tarea 7 — probar Jazzy en paralelo, sin tocar Humble

**Por qué**: Humble tiene soporte hasta mayo de 2027 y Jazzy hasta mayo de
2029. El mini PC aún no está instalado, así que ahora cambiar sale barato.
**No está decidido**: se trata de probarlo y apuntar qué pasa; decide Mario
(issue #15). Hacerla **después de la tarea 6**, para que un fallo en Jazzy no se
confunda con uno del código nuevo.

Reglas:

- No tocar la distro `Ubuntu-22.04` ni su `~/microros_ws`. Jazzy va en una
  distro nueva, `Ubuntu-24.04` (antes, `wsl -l -v`).
- El código tiene que seguir compilando en Humble. En la app del ESP32, lo que
  cambie va por versión de ESP-IDF en `idf_compat.h`.
- Solo el ESP32 pelado del banco, nunca el robot.
- No cambiar el `humble` por defecto de los scripts ni los docs hasta que
  Mario decida.

Pasos:

1. `wsl --install -d Ubuntu-24.04`. Dentro: `git config --global
   protocol.version 0`, clonar `feature/ros2` y, desde una terminal
   interactiva, `./scripts/instalar_pc_abordo.sh jazzy`.
2. `colcon build` y `colcon test`: 116 tests, ninguno saltado. Los avisos de
   setuptools por Python 3.12 son normales.
3. La tarea 6 otra vez, en Jazzy.
4. `micro_ros_setup` en su rama `jazzy`. **Apuntar qué ESP-IDF trae.** Donde
   espero que salte algo:
   - `app.c` reutiliza `esp32_serial_open/close/write/read` de
     `freertos_apps` para subir el UART a 921600. Si cambian de nombre o de
     firma, adaptarlo. Es lo primero que hay que mirar.
   - Con ESP-IDF 5.x, el timer (`driver/timer.h`) y el I2C (`driver/i2c.h`)
     de `hw.c` son drivers antiguos: deberían compilar con avisos. No
     migrarlos salvo que no compile.
5. Flashear el ESP32 pelado y repetir las pruebas de la sección 2.
6. El mando por el puente UDP.

Al acabar, dejar aquí una tabla Humble frente a Jazzy paso por paso
(funciona / con avisos / roto y qué se tocó), la versión de ESP-IDF y cuánto
llevó, y proponerle a Mario un resumen para la #15.

---

## 5. Decisiones cerradas — no las vuelvas a abrir

- **Sin LiDAR ni mapeo.** Se quitaron a propósito; nav2 no pinta nada todavía.
- **CAN del ESP32: TWAI + transceptor SN65HVD230**, no MCP2515 (issue #7).
- **Gazebo, descartado por ahora**: no modela orugas, y `starcrawler_sim` basta.
- **Mallas CAD del URDF bloqueadas** (falta FreeCAD): primitivas mientras tanto.
- **Los firmwares de `firmware/` siguen en Arduino core.** La app de
  `micro_ros_esp32_apps/` es ESP-IDF porque lo pide micro-ROS.

Si crees de verdad que alguna hay que reabrirla, dilo y que decida Mario.

---

## 6. Normas

- **Commits sencillos**: asunto y dos o tres líneas. **Sin línea de coautor.**
  Un commit por causa arreglada, no todo junto.
- **No commitear notas ni docs por iniciativa propia.** El razonamiento va a
  los `.md` que se pidan y a las issues, no a comentarios en el código.
- No tocar `main`: está congelada y se arregla al mergear.
- No dar por verificado lo que solo compila.

---

## 7. Mapa del repo

```
ros2_ws/src/
  starcrawler_msgs/         CrawlerCommand y RobotState (CMake)
  starcrawler_common/       conversión de ángulos, única fuente de verdad
  starcrawler_driver/       puente serie con CRC16 (el camino anterior)
  starcrawler_sim/          robot simulado a nivel de tópicos
  starcrawler_odometry/     odometría de orugas y TF odom -> base_footprint
  starcrawler_teleop/       mando DS4, joy_udp_node y twist_mux
  starcrawler_gui/          interfaz web: :8000 y :8000/3d
  starcrawler_description/  URDF y configuraciones de RViz
  starcrawler_bringup/      launch, systemd y udev

micro_ros_esp32_apps/starcrawler_app/   el ESP32 como nodo ROS 2
tools/joy_bridge/                       el mando de Windows al WSL
scripts/                                instalación y arranque del mini PC
docs/ros2.md                            arquitectura y puesta en marcha
```

El launch elige de dónde sale el estado del robot, y son excluyentes: el
agente de micro-ROS (`micro_ros:=true`, el camino actual), el simulador
(`sim:=true`) o el driver serie (por defecto, el camino anterior).

---

## 8. Issues que tocan esto

- **#15** — Humble o Jazzy (tarea 7).
- **#16** — migrar el ESP32 a micro-ROS: compila y funciona en el banco.
- **#13** — calibrar geometría y mapeo del DS4.
- **#21** — el bus CAN se cae con los cuatro motores (hardware, bloqueante).
- **#7** — transceptor CAN del ESP32.
- **#12** — primer `colcon build`: hecho, falta cerrarla.

Todo en `github.com/star-uma/StarCrawler/issues`.
