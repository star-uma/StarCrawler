# Entorno en Windows para los firmwares de Arduino

Lo necesario para compilar y flashear desde Windows los firmwares de
`firmware/` y los sketches de prueba. **No hace falta para ROS 2**: eso va en
Ubuntu, ver [`ros2.md`](ros2.md).

## Requisitos

| Tool | Version | Notes |
|---|---|---|
| Python | 3.12 (recommended) | 3.15 has compatibility issues with pygame |
| Git | 2.53+ | |
| VS Code | 1.116+ | |
| Arduino CLI | 1.4.1 | Manually installed at `C:\arduino-cli\` |

### Extensiones de VS Code

| Extension | ID |
|---|---|
| Arduino Community Edition | `vscode-arduino` |
| Python | `ms-python.python` |
| Pylance | `ms-python.vscode-pylance` |
| GitLens | `eamodio.gitlens` |
| Error Lens | `usernamehakobyan.error-lens` |
| Serial Monitor | `ms-vscode.serial-monitor` |

## Instalación desde cero

### 1. Python

Download Python 3.12 from https://www.python.org/downloads/release/python-3128/
During installation mark: ☑️ **Add python.exe to PATH**

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

```powershell
arduino-cli version
```

### 3. Placas y librerías

**Lo más rápido es el script** de la rama `feature/test-target-bringup`, que
instala lo necesario y comprueba al final que todos los sketches compilan:

```powershell
./test/target/instalar_entorno.ps1
```

A mano sería:

```powershell
arduino-cli config add board_manager.additional_urls `
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32      # firmware actual
arduino-cli core install arduino:samd     # solo para el MKR de la v1
arduino-cli lib install "ACAN2515"
arduino-cli lib install "CAN"
```

> El firmware **standalone** necesita además otro board package, porque
> Bluepad32 sustituye la pila Bluetooth. Está en `docs/standalone_sin_pc.md`
> de la rama `feature/standalone-sin-pc`.

Comprobar que se detecta la placa (conectada por USB):

```powershell
arduino-cli board list
```

### 4. Clave SSH para GitHub

```powershell
ssh-keygen -t ed25519 -C "your_email@gmail.com"
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

Copy the output and add it to GitHub → Settings → SSH and GPG keys → New SSH key → **Authentication Key**.

```powershell
ssh -T git@github.com
```

### 5. Identidad de git

```powershell
git config --global user.email "your_email@gmail.com"
git config --global user.name "your_username"
```

### 6. Clonar

```powershell
git clone -b feature/ros2 git@github.com:star-uma/StarCrawler.git
```

## Compilar y flashear

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 firmware/starcrawler_esp32
```

Cambiar `starcrawler_esp32` por la variante que toque:
`starcrawler_esp32_basico` (sin IMU) o `starcrawler_esp32_ros2` (esclavo serie
de ROS 2, el camino anterior a micro-ROS). La app de micro-ROS no se compila
así: ver `micro_ros_esp32_apps/starcrawler_app/README.md`.

Tests de lógica pura del firmware, sin hardware:

```powershell
./test/host/run_tests.ps1
```

## Git, lo básico

```powershell
git checkout -b feature/nombre
git push -u origin feature/nombre
```

```powershell
git add .
git commit -m "tipo(ambito): descripcion"
git push
```
