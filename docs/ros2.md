# StarCrawler en ROS 2

Arquitectura con **PC a bordo**: ROS 2 corre en el robot y el ESP32 se queda
como capa de tiempo real. Este documento explica el planteamiento, cómo
instalar ROS 2 desde cero y cómo arrancar el sistema (con y sin hardware).

Rama: `feature/ros2`. Workspace: [`ros2_ws/`](../ros2_ws).

---

## 1. Arquitectura

```
        [Mando DS4] ──Bluetooth──┐
                                 ▼
┌──────────────────────────────────────────────────────────────┐
│  PC A BORDO — Ubuntu 24.04 + ROS 2 Jazzy                     │
│                                                              │
│   joy_node ──/joy──► starcrawler_teleop ──/cmd_vel───────┐   │
│                                       └──/crawler/command┤   │
│                                                          ▼   │
│                                          starcrawler_driver  │
│   robot_state_publisher ◄──/joint_states──────┤               │
│   rviz2 / rosbag / (Nav2 a futuro) ◄──/starcrawler/state      │
│                                        /diagnostics          │
└───────────────────────────────┬──────────────────────────────┘
                                │ USB serie 921600, tramas con CRC16
                                ▼
                    ┌───────────────────────────┐
                    │  ESP32 (tiempo real)      │
                    │  watchdog 300 ms          │
                    ├───────────────────────────┤
                    │ CAN 1 Mbps ─► 4x RMD-X8   │  tracción
                    │ GPIO ──────► 4x DM542     │  elevación
                    │ I2C ───────► TCA9548A +   │
                    │              4x AS5600    │  encoders
                    └───────────────────────────┘
```

### ¿Por qué el ESP32 se queda?

Con un PC a bordo se podría poner el CAN en el PC (existe el driver
open-source `myactuator_rmd` con integración `ros2_control`) y ahorrarse el
micro. No lo recomiendo todavía, por dos razones concretas:

1. **Los pulsos de los steppers necesitan tiempo real.** La generación del
   tren de STEP es una ISR a 833 Hz. Un Linux sin kernel PREEMPT_RT tiene
   jitter de milisegundos: se traduce en pasos perdidos y ruido mecánico.
2. **La seguridad no debe depender del PC.** Si el PC se cuelga, se
   desconecta el USB o se cierra el nodo, el watchdog del ESP32 libera los
   motores en 300 ms. Es una capa independiente y ya está probada.

Reparto de responsabilidades:

| | PC a bordo | ESP32 |
|---|---|---|
| Mando, modos, presets | ✅ | |
| Cinemática, odometría, TF | ✅ | |
| Navegación, mapas, cámaras (futuro) | ✅ | |
| Pulsos de stepper, lazo de posición | | ✅ |
| Tramas CAN espaciadas, límites software | | ✅ |
| Watchdog y parada segura | (2ª capa) | ✅ **manda** |

### Convención de ángulos (importante)

Hay dos espacios y una única traducción, en `protocol.py`:

- **Grados de encoder** (bus serie y firmware): 180° = oruga horizontal, y las
  orugas FL y RR van espejadas (`360-a`), como documenta el TFG.
- **Radianes de elevación** (todo ROS): **positivo = brazo levantado**, 0 =
  horizontal, igual en las cuatro orugas.

En elevación las poses son simétricas: horizontal = `{0,0,0,0}`, vertical
arriba = `{+90°,+90°,+90°,+90°}` (que en encoder es el `{90,270,270,90}` del
TFG). Eso simplifica el URDF, la teleoperación y RViz — y corrige de paso el
bug latente del modo 2 original, que mandaba el mismo ángulo a las cuatro.

---

## 2. Qué PC llevar a bordo

| Opción | Precio aprox. | Por qué |
|---|---|---|
| **Mini PC x86 (Intel N100/N150)** | 150–250 € | **Recomendado.** ROS 2 sin sorpresas, x86 = todo binario disponible, 6–12 W, entrada 12 V que sale directa de la batería con un DC-DC |
| Raspberry Pi 5 (8 GB) | ~90 € | Barata y ligera; Ubuntu 24.04 va bien. Justa si luego añades visión |
| Jetson Orin Nano | 250–500 € | Solo si el plan incluye visión/IA a bordo |

Detalles de integración: la batería del TFG es de Li-ion a medida, así que
hace falta un **DC-DC regulado** (12 V para el mini PC, 5 V/5 A para una Pi) y
un interruptor propio para el PC — apagar en caliente un PC repetidamente
corrompe el sistema de ficheros. Reserva también un USB para el ESP32.

---

## 3. Instalar ROS 2 (Ubuntu 24.04 + Jazzy)

**Distribución elegida: ROS 2 Jazzy Jalisco** sobre **Ubuntu 24.04 LTS**. Es
la LTS con soporte hasta 2029; es lo que quieres en un robot que va a durar.

> **Nota WSL / Ubuntu 22.04:** si desarrollas en un WSL con Ubuntu 22.04
> (jammy), instala **ROS 2 Humble** en su lugar: mismos pasos cambiando
> `jazzy` por `humble` y `$UBUNTU_CODENAME` resuelve a `jammy`. Todos los
> paquetes de StarCrawler funcionan igual en ambas. Humble tiene soporte
> hasta mayo de 2027; para el PC definitivo del robot, mejor 24.04 + Jazzy.

> Los pasos exactos de instalación cambian de vez en cuando (el repositorio de
> ROS pasó a distribuirse con un paquete `ros2-apt-source`). Si algo falla,
> la referencia buena es <https://docs.ros.org/en/jazzy/Installation.html>.

```bash
# 1. Locale UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. Repositorio de ROS 2
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. ROS 2 Jazzy + herramientas de compilación
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 4. Paquetes que usa StarCrawler
sudo apt install -y \
    ros-jazzy-joy ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
    ros-jazzy-diagnostic-updater python3-serial

# 5. Cargar ROS en cada terminal (y dejarlo en el .bashrc)
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Comprobación rápida (dos terminales):

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### Compilar el workspace de StarCrawler

```bash
git clone -b feature/ros2 https://github.com/star-uma/StarCrawler.git
cd StarCrawler/ros2_ws

sudo rosdep init          # solo la primera vez en la máquina
rosdep update
rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

`--symlink-install` hace que los cambios en ficheros Python se apliquen sin
recompilar (muy cómodo mientras se ajustan parámetros).

---

## 4. Probar sin hardware (hazlo primero)

El driver incluye un **ESP32 simulado** que integra un modelo del robot
(velocidad real de los steppers, límites de recorrido, watchdog). Sirve para
validar todo el grafo ROS y ver el robot moverse en RViz sin tocar el robot:

```bash
ros2 launch starcrawler_bringup robot.launch.py simulate:=true rviz:=true
```

Y para mover las orugas a mano y comprobar el URDF y los signos:

```bash
ros2 launch starcrawler_description view_model.launch.py
```

Comandos útiles para inspeccionar:

```bash
ros2 topic list
ros2 topic echo /starcrawler/state          # ángulos, errores, seguridad
ros2 topic hz /joint_states                 # debe rondar 50 Hz
ros2 topic echo /diagnostics
ros2 run rqt_graph rqt_graph                # ver el grafo de nodos
ros2 bag record -a -o sesion_01             # grabar TODO para revisarlo luego
```

Mover el robot sin mando, para probar la cadena:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.01}, angular: {z: 0.0}}'

ros2 topic pub -r 20 /crawler/command starcrawler_msgs/msg/CrawlerCommand \
  '{increment: [1, 1, 0, 0]}'               # sube el par delantero
```

---

## 5. Puesta en marcha con el robot

### 5.1 Flashear el ESP32

Firmware nuevo: [`firmware/starcrawler_esp32_ros2/`](../firmware/starcrawler_esp32_ros2).
Sin WiFi ni Bluetooth, así que se compila con el **core esp32 normal** (no el
de Bluepad32) y ocupa solo el 22 % de la flash.

```bash
arduino-cli lib install ACAN2515
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_ros2
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_ros2
```

### 5.2 Nombre estable para el puerto serie

Sin esto el ESP32 baila entre `/dev/ttyUSB0` y `/dev/ttyUSB1`:

```bash
cd ros2_ws/src/starcrawler_bringup/udev
# 1. averigua el chip de tu placa y descomenta la línea que toque
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
nano 99-starcrawler.rules
# 2. instalar
sudo cp 99-starcrawler.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER    # cerrar sesión y volver a entrar
ls -l /dev/starcrawler            # debe existir
```

### 5.3 Emparejar el mando DS4 en Linux

```bash
bluetoothctl
> power on
> agent on
> scan on
# en el mando: SHARE + PS unos 5 s, hasta doble parpadeo blanco
> pair  XX:XX:XX:XX:XX:XX
> trust XX:XX:XX:XX:XX:XX
> connect XX:XX:XX:XX:XX:XX
> quit
```

**Verifica el mapeo antes de conducir** (los índices cambian entre mandos y
drivers):

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Pulsa cada botón, anota su índice y ajústalo en
[`ros2_ws/src/starcrawler_teleop/config/ds4.yaml`](../ros2_ws/src/starcrawler_teleop/config/ds4.yaml).
No hay que tocar código.

### 5.4 Arrancar

```bash
ros2 launch starcrawler_bringup robot.launch.py
```

Opciones: `port:=/dev/ttyUSB0`, `teleop:=false` (solo driver), `rviz:=true`,
`simulate:=true`.

Para ver el robot desde otro ordenador de la misma red:

```bash
export ROS_DOMAIN_ID=42          # el mismo en robot y portátil
ros2 launch starcrawler_bringup rviz.launch.py
```

Arranque automático al encender: ver
[`systemd/starcrawler.service`](../ros2_ws/src/starcrawler_bringup/systemd/starcrawler.service)
(habilítalo solo cuando el mapeo esté verificado).

---

## 6. Mando (esquema simultáneo)

Es el mismo esquema del firmware standalone: tracción siempre activa y orugas
superpuestas.

| Control | Función |
|---|---|
| Stick izq. ↕ / der. ↔ | Avanzar / girar — **siempre activos** |
| L1 / L2 | Par **delantero** sube / baja |
| R1 / R2 | Par **trasero** sube / baja |
| Cruceta | Inclinar el conjunto |
| ✕ ○ □ △ (mantener 0.5 s) | Poses: −45° / 0° / +45° / +90° de elevación |
| **SHARE** | **Parada de emergencia** |
| L3 | Velocidad lenta (×0.5) / rápida |
| OPTIONS | Reservado (nivelado, requiere IMU) |

---

## 7. Calibración pendiente

Tres cosas que el código no puede resolver solo:

1. **`wheel_radius` y `track_separation`** (en `config/driver.yaml`). Solo
   afectan a la conversión `/cmd_vel` ↔ dps y a la odometría. Método honesto:
   publica `linear.x = 0.01 m/s` durante 10 s y mide lo recorrido; ajusta el
   radio en proporción. La separación, con cinta métrica.
2. **Mapeo del mando** (`config/ds4.yaml`) — ver 5.3.
3. **Offsets de encoder** `{-2, -5, 16, -15}` en el `config.h` del firmware:
   son del robot en 2025, conviene recalibrarlos con las orugas en horizontal.

Y una verificación de hardware que sigue pendiente de las etapas anteriores:
la **tabla de signos de la compensación de tracción**
(`TABLA_SIGNO_COMPENSACION`), con el robot sobre tacos.

---

## 8. Seguridad

En capas, de dentro a fuera:

1. **ESP32 (la que manda)**: watchdog de 300 ms sin trama válida → libera
   tracción y detiene elevación. Límites software de recorrido (85°–275°).
   Sin encoder válido, el lazo de posición de esa oruga se inhibe.
2. **Driver del PC**: si `/cmd_vel` o `/crawler/command` se quedan viejos
   (>0.5 s) manda ceros explícitos.
3. **Teleop**: SHARE → flag de emergencia dedicado; sin `/joy` manda parada.
   Hay un *deadman* opcional (`boton_enable`, desactivado por defecto).

Las tramas llevan CRC16 y contadores de error, expuestos en `/diagnostics`:
si el cable USB da problemas, se ve en `frames_crc_error`.

---

## 9. Estado y hoja de ruta

**Verificado en este PC (sin robot):**

| Comprobación | Resultado |
|---|---|
| Protocolo serie, C ↔ Python byte a byte | ✅ 26/26 (C) + 25/25 (Python) |
| Lógica de control del firmware | ✅ 66/66 |
| Lógica del mando | ✅ 27/27 |
| Firmware ESP32 compila | ✅ 22 % flash |
| xacro → URDF, estructura y árbol | ✅ 7 links, 6 joints, raíz única |
| Convención de signos en el URDF | ✅ las 4 orugas suben con valor positivo |
| package.xml / YAML / sintaxis Python | ✅ |

**Sin verificar (necesita Ubuntu o el robot):** `colcon build` completo,
arranque real de los nodos, y todo lo del apartado 7.

**Siguientes pasos naturales:**

1. **IMU en el PC** (USB/I2C) + `imu_filter_madgwick` → recuperar el modo de
   nivelado automático del TFG en el lado ROS. Con PC a bordo es mejor que
   colgarla del ESP32.
2. **`ros2_control`**: sustituir el nodo driver por un `SystemInterface`, con
   `diff_drive_controller` (odometría estándar) y `position_controllers` para
   las orugas.
3. **Nav2**: con `/cmd_vel`, odometría fusionada y un LiDAR 2D barato, el
   robot navega solo. Es donde ROS 2 empieza a pagar de verdad.
4. **micro-ROS en el ESP32** si se quiere que el micro sea un nodo ROS nativo
   (ojo: sustituye la pila de comunicaciones del firmware).

Nota sobre reutilización: `control_core` y `proto` son C puro sin
dependencias, y `protocol.py` / `joy_logic.py` no importan rclpy. Es decir,
la lógica probada se puede llevar a micro-ROS o a `ros2_control` sin
reescribir nada.
