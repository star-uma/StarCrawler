# StarCrawler

Refactoring of the **Horu** robot control system — renamed **StarCrawler**. Migrates from Simulink + Arduino IDE to a fully version-controlled Python + VS Code environment.

> **Para retomar el trabajo, o para abrir una sesión de Claude en este repo:**
> [`README-CLAUDE.md`](README-CLAUDE.md) tiene el estado real del proyecto —qué
> está verificado y qué no— y las tareas en orden. Las normas de trabajo están
> en [`CLAUDE.md`](CLAUDE.md), que Claude Code carga solo.

> **v2 en desarrollo — arquitectura unificada:** todo el control (tracción CAN,
> elevación, IMU, WiFi) pasa a un único ESP32, eliminando el Arduino MKR.
> Dos variantes: `firmware/starcrawler_esp32/` (completa, modos 1-5 con IMU) y
> `firmware/starcrawler_esp32_basico/` (solo tracción + elevación, sin IMU).
> Tests en `test/` y documentación completa en
> [docs/arquitectura_esp32_unificada.md](docs/arquitectura_esp32_unificada.md).
> El firmware MKR de abajo queda como referencia de la arquitectura v1.
>
> **v3 en esta rama — ROS 2 con PC a bordo:** workspace completo en
> `ros2_ws/` (driver serie, teleop, URDF, launch) + firmware esclavo
> `firmware/starcrawler_esp32_ros2/`. Instalación de ROS 2, arquitectura y
> puesta en marcha en **[docs/ros2.md](docs/ros2.md)**.
>
> ```powershell
> # Tests unitarios (sin hardware)
> ./test/host/run_tests.ps1
> # Compilar firmware unificado
> arduino-cli lib install ACAN2515
> arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
> ```

## What is StarCrawler?

StarCrawler is a tracked robot with four independently articulated crawler arms. Each arm is driven by a stepper motor (elevation) and a brushless motor (traction), giving the robot the ability to traverse obstacles and self-level on uneven terrain.

## Estructura del repositorio

```
StarCrawler/
├── firmware/
│   ├── starcrawler_esp32/          ESP32 unificado, modos 1-5 con IMU
│   ├── starcrawler_esp32_basico/   igual pero sin IMU
│   ├── starcrawler_esp32_ros2/     esclavo de ROS 2 (serie con CRC16)
│   └── starcrawler_mkr_traccion/   arquitectura v1, solo referencia
├── micro_ros_esp32_apps/
│   └── starcrawler_app/            el ESP32 como nodo ROS 2 nativo
├── ros2_ws/src/                    nueve paquetes: driver, sim, teleop,
│                                   odometria, gui, descripcion, msgs,
│                                   comun y bringup
├── test/
│   ├── target/                     puesta en marcha con el robot delante
│   ├── host/                       tests de logica pura, sin hardware
│   └── hil/                        sistema completo por UDP
├── scripts/                        instalacion y arranque del PC de a bordo
└── docs/                           arquitectura, ROS 2, cadena de elevacion
```

## Arquitectura

```
        [Mando] ──► PC a bordo (Ubuntu + ROS 2)
                         │  USB serie
                         ▼
                    [ESP32 unico]
                    ├── CAN 1 Mbps ──► 4x RMD-X8      traccion
                    ├── GPIO x12 ────► 4x DM542       elevacion
                    └── I2C ─────────► TCA9548A + 4x AS5600  encoders
                                       MPU9250 (IMU)
```

Un solo microcontrolador para todo. El Arduino MKR de la v1 se elimina: solo
aportaba WiFi (que el ESP32 ya tiene), el bus CAN (que cubre el TWAI interno) y
la IMU (que se recablea).

**El detalle de cada cosa está en `docs/`:**

| Documento | De qué va |
|---|---|
| [`docs/ros2.md`](docs/ros2.md) | Arquitectura ROS 2, instalación y puesta en marcha |
| [`docs/arquitectura_esp32_unificada.md`](docs/arquitectura_esp32_unificada.md) | Por qué cabe todo en un ESP32 y qué cambia del cableado |
| [`docs/cadena_de_elevacion.md`](docs/cadena_de_elevacion.md) | De dónde sale cada constante de los steppers |
| [`test/target/README.md`](test/target/README.md) | Cómo probar cada componente por separado |

## Protocolo UDP de la v1 (referencia)

| Index | Field | Description |
|---|---|---|
| 0 | `id` | Always 1 (PC origin) |
| 1 | `action_left_train` | Left traction speed (dps × 100) |
| 2 | `action_right_train` | Right traction speed (dps × 100) |
| 3–6 | `o0..o3` | Crawler target angles / directions (modes 2–4) |
| 7 | `mode` | Active control mode (1..5) |
| 8 | `code_error` | 0 = no error |

## Control modes

| Mode | Name | Description |
|---|---|---|
| 1 | Traction | Differential drive — left stick (speed) + right stick (turn) |
| 2 | Absolute position | Set all crawlers to a fixed angle via A/B/X/Y buttons |
| 3 | Incremental × 4 | Tilt all crawlers together via D-pad |
| 4 | Incremental × 2 | Control crawler pairs independently via D-pad + triggers |
| 5 | Auto-levelling | IMU-based automatic horizontal levelling (hold Start) |

## Mando en la v1 (referencia)

| Input | Action |
|---|---|
| Left stick (vertical) | Forward / backward |
| Right stick (horizontal) | Turn |
| RB | Cycle mode: 1 → 2 → 3 → 4 → 5 → 1 |
| A / B / X / Y | Absolute position (Mode 2): 225° / 180° / 135° / 90° |
| D-pad + triggers | Crawler control (Modes 3 / 4) |
| Start (hold) | Auto-levelling (Mode 5) |

---

## Development environment setup

### Requirements

| Tool | Version | Notes |
|---|---|---|
| Python | 3.12 (recommended) | 3.15 has compatibility issues with pygame |
| Git | 2.53+ | |
| VS Code | 1.116+ | |
| Arduino CLI | 1.4.1 | Manually installed at `C:\arduino-cli\` |

### VS Code extensions

| Extension | ID |
|---|---|
| Arduino Community Edition | `vscode-arduino` |
| Python | `ms-python.python` |
| Pylance | `ms-python.vscode-pylance` |
| GitLens | `eamodio.gitlens` |
| Error Lens | `usernamehakobyan.error-lens` |
| Serial Monitor | `ms-vscode.serial-monitor` |

---

## Installation from scratch (Windows)

### 1. Python

Download Python 3.12 from https://www.python.org/downloads/release/python-3128/
During installation mark: ☑️ **Add python.exe to PATH**

Install dependencies:
```powershell
python -m pip install pygame
```

### 2. Arduino CLI

Download the Windows 64-bit ZIP from https://arduino.github.io/arduino-cli/latest/installation/

```powershell
mkdir C:\arduino-cli
Copy-Item "path\to\arduino-cli.exe" "C:\arduino-cli\arduino-cli.exe"
$env:Path += ";C:\arduino-cli"
[Environment]::SetEnvironmentVariable("Path", [Environment]::GetEnvironmentVariable("Path","User") + ";C:\arduino-cli", "User")
```

Verify:
```powershell
arduino-cli version
```

### 3. Placas y librerias

**Lo mas rapido es el script**, que instala lo necesario y comprueba al final
que todos los sketches compilan:

```powershell
./test/target/instalar_entorno.ps1
```

A mano seria:

```powershell
arduino-cli config add board_manager.additional_urls `
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32      # firmware actual
arduino-cli core install arduino:samd     # solo para el MKR de la v1
arduino-cli lib install "ACAN2515"
arduino-cli lib install "CAN"
```

> El firmware **standalone** necesita ademas otro board package, porque
> Bluepad32 sustituye la pila Bluetooth. Esta en
> [`docs/standalone_sin_pc.md`](docs/standalone_sin_pc.md).

Comprueba que se detecta la placa (conectala por USB primero):

```powershell
arduino-cli board list
```

### 4. SSH key for GitHub

```powershell
ssh-keygen -t ed25519 -C "your_email@gmail.com"
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

Copy the output and add it to GitHub → Settings → SSH and GPG keys → New SSH key → **Authentication Key**.

Verify:
```powershell
ssh -T git@github.com
```

### 5. Git identity

```powershell
git config --global user.email "your_email@gmail.com"
git config --global user.name "your_username"
```

### 6. Clone the repository

```powershell
git clone -b main git@github.com:star-uma/StarCrawler.git
```

---

## Como se arranca

### Con ROS 2 (esta rama)

Todo el detalle esta en [`docs/ros2.md`](docs/ros2.md). En corto:

```bash
# sin nada de hardware
ros2 launch starcrawler_bringup robot.launch.py sim:=true rviz:=true gui:=true

# con el robot
ros2 launch starcrawler_bringup robot.launch.py
```

### Compilar y flashear el ESP32

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_ros2
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32_ros2
```

Cambia `starcrawler_esp32_ros2` por la variante que toque: `starcrawler_esp32`
(completa, con IMU) o `starcrawler_esp32_basico` (sin IMU).

### Probar componentes por separado

Antes de flashear el firmware completo a un robot recien montado, conviene
verificar cada subsistema. Esta todo en
[`test/target/`](test/target/README.md).

### El control por PC de la v1

```powershell
cd control
python StarCrawlerXbox.py --test
```

Manda datagramas UDP al firmware de la v1. Se mantiene como referencia.

---

## Git workflow

### Create a new feature branch

```powershell
git checkout -b feature/branch-name
git push -u origin feature/branch-name
```

### Daily workflow

```powershell
git add .
git commit -m "type(scope): description"
git push
```

### Branch strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, tested code |
| `dev` | Integration branch |
| `feature/*` | New features or modes |
| `fix/*` | Bug fixes |

---

## Safety

The Arduino firmware includes a **500 ms watchdog**: if no UDP packet is received within that window (connection lost, script stopped), all traction motors are released immediately.