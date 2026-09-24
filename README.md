# StarCrawler

Robot de orugas con cuatro brazos articulados, de la UMA. Cada brazo lleva un
motor de tracción y uno de elevación, para salvar obstáculos y nivelarse en
terreno irregular. Es el sucesor del robot Horu.

> **¿Vas a abrir Claude en este repo?** Empieza por
> [`README-CLAUDE.md`](README-CLAUDE.md): qué está verificado y qué no, y las
> tareas en orden.

## Arquitectura

```
   [Mando DS4] ──► PC a bordo (Ubuntu 22.04 + ROS 2 Humble)
                         │  USB serie, micro-ROS
                         ▼
                   [ESP32 único]
                   ├── CAN 1 Mbps ──► 4× RMD-X8     tracción
                   ├── GPIO ×12 ────► 4× DM542      elevación (paso a paso)
                   └── I2C ─────────► TCA9548A + 4× AS5600   encoders
                                      MPU9250 (IMU)
```

El ESP32 es un nodo más de ROS 2 (micro-ROS) y se encarga del tiempo real. El
PC de a bordo lleva el resto: teleoperación, odometría e interfaz web.

## Arrancarlo

Sin nada de hardware, con el robot simulado:

```bash
ros2 launch starcrawler_bringup robot.launch.py sim:=true gui:=true
```

Con el ESP32 conectado:

```bash
ros2 launch starcrawler_bringup robot.launch.py micro_ros:=true port:=/dev/ttyUSB0 gui:=true
```

La interfaz queda en `http://localhost:8000` (telemetría) y
`http://localhost:8000/3d` (el robot moviéndose por el plano).

| Para | Ver |
|---|---|
| Instalar ROS 2 y compilar | [`docs/ros2.md`](docs/ros2.md) |
| Flashear el ESP32 | [`micro_ros_esp32_apps/starcrawler_app/README.md`](micro_ros_esp32_apps/starcrawler_app/README.md) |
| Usar el mando desde Windows (WSL no lo ve) | [`tools/joy_bridge/README.md`](tools/joy_bridge/README.md) |
| Montar el mini PC del robot | [`docs/instalar_ubuntu_pc_abordo.md`](docs/instalar_ubuntu_pc_abordo.md) |

## Ramas

| Rama | Para qué |
|---|---|
| `feature/ros2` | El tronco: ROS 2, micro-ROS y el simulador. **Aquí se trabaja** |
| `feature/test-target-bringup` | Probar cada componente por separado, con el robot delante |
| `feature/standalone-sin-pc` | El ESP32 solo, con el mando por Bluetooth y sin PC |
| `main` | Congelada en julio, arquitectura v1. Se pone al día al mergear |

## Qué hay en el repo

```
ros2_ws/src/            los nueve paquetes de ROS 2
micro_ros_esp32_apps/   el firmware del ESP32 como nodo ROS 2
firmware/               los firmwares de Arduino: unificado, básico,
                        esclavo serie y el MKR de la v1
tools/joy_bridge/       el mando de Windows al WSL por UDP
scripts/                instalación y arranque del PC de a bordo
test/host/, test/hil/   tests sin hardware y del sistema completo
control/                el control por PC de la v1
docs/                   todo lo demás, ver abajo
```

## Documentación

| Documento | De qué va |
|---|---|
| [`docs/ros2.md`](docs/ros2.md) | Arquitectura ROS 2, instalación, puesta en marcha y seguridad |
| [`docs/arquitectura_esp32_unificada.md`](docs/arquitectura_esp32_unificada.md) | Por qué cabe todo en un ESP32 y qué cambia del cableado |
| [`docs/cadena_de_elevacion.md`](docs/cadena_de_elevacion.md) | De dónde sale cada constante de los paso a paso |
| [`docs/entorno_windows.md`](docs/entorno_windows.md) | Arduino CLI en Windows para los firmwares de `firmware/` |
| [`docs/v1_referencia.md`](docs/v1_referencia.md) | La arquitectura anterior: MKR, protocolo UDP y modos |
