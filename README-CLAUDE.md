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
| Los 9 paquetes de `ros2_ws/` | **Nunca se ha hecho `colcon build`** |
| `micro_ros_esp32_apps/starcrawler_app/` | **Nunca se ha compilado** |
| Los 101 tests de lógica pura | Nunca se han pasado con `colcon test`; los que se han corrido a mano pasan |
| Cualquier cosa contra el robot | Nada verificado en hardware |

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

- **El mando.** WSL2 no expone `/dev/input/js0`. Hay que lanzar con
  `teleop:=false` y mover el robot simulado publicando `/cmd_vel` a mano o con
  `teleop_twist_keyboard`.
- **El ESP32 por serie.** Requiere `usbipd-win`, y mientras el puerto está
  bindeado a WSL desaparece el COM de Windows. Para compilar y flashear
  firmware es más cómodo el Arduino IDE en Windows, sin WSL de por medio.

---

## 4. Lo que toca hacer, en orden

### Tarea 1 — elegir distribución de ROS 2 (issue #15)

Condiciona la versión de Ubuntu, así que va primero.

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

### Tarea 2 — instalar y compilar (issue #12) ← lo importante

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

### Tarea 3 — pasar los tests

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

### Tarea 4 — levantar el robot simulado

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

### Tarea 5 — compilar la app de micro-ROS

`micro_ros_esp32_apps/starcrawler_app/` es el ESP32 como nodo ROS 2 nativo. El
CAN, los paso a paso y los encoders se portaron a ESP-IDF **a ciegas**, así que
es lo que más probable es que esté roto.

Necesita ESP-IDF y `micro_ros_setup`, que es una instalación más pesada. No
hace falta el robot para compilarlo. Instrucciones en
`micro_ros_esp32_apps/starcrawler_app/README.md`.

---

## 5. Lo que no hay que hacer

- **No commitear notas ni docs por iniciativa propia.** El razonamiento va a
  los `.md` que se pidan y a las issues, no a comentarios en el código.
- **Commits sencillos**: asunto y dos o tres líneas como mucho. **Sin línea de
  coautor.**
- No tocar `main`: está congelado y se arregla cuando se mergee.
- No dar por verificado lo que solo compila.

---

## 6. Mapa del repo

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

## 7. Issues abiertas que tocan esto

- **#12** — primer `colcon build` del workspace.
- **#15** — decidir Humble o Jazzy.
- **#21** — el bus CAN se cae con los cuatro motores (hardware, bloqueante).
- **#7** — transceptor CAN del ESP32.
- **#6** — la fórmula de compensación de tracción.

Todo en `github.com/star-uma/StarCrawler/issues`.
