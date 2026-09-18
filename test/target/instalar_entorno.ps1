<#
    instalar_entorno.ps1 - StarCrawler
    ==================================================================

    Deja un PC con Windows listo para compilar y subir los sketches de
    test/target/ al robot. Pensado para un PC limpio: no hay que saber
    nada previo.

    QUE INSTALA
      - arduino-cli (si no esta ya)
      - El soporte de placa del ESP32      -> pruebas 1, 3 y 4
      - El soporte de placa del Arduino MKR -> prueba 2
      - La libreria CAN                     -> prueba 2

    COMO SE USA
      Abre PowerShell en la carpeta del repo y ejecuta:

          .\test\target\instalar_entorno.ps1

      Si Windows se queja de permisos para ejecutar scripts:

          powershell -ExecutionPolicy Bypass -File .\test\target\instalar_entorno.ps1

    Tarda un rato la primera vez (el soporte del ESP32 son varios cientos
    de MB). Se puede volver a ejecutar sin problema: lo que ya este
    instalado se lo salta.

    ==================================================================
#>

$ErrorActionPreference = "Stop"

function Titulo($texto) {
    Write-Host ""
    Write-Host "==================================================" -ForegroundColor Cyan
    Write-Host "  $texto" -ForegroundColor Cyan
    Write-Host "==================================================" -ForegroundColor Cyan
}

function Paso($texto)  { Write-Host "  -> $texto" }
function Bien($texto)  { Write-Host "  [OK] $texto"    -ForegroundColor Green }
function Aviso($texto) { Write-Host "  [!]  $texto"    -ForegroundColor Yellow }
function Malo($texto)  { Write-Host "  [ERROR] $texto" -ForegroundColor Red }

Titulo "StarCrawler - instalacion del entorno de pruebas"
Write-Host ""
Write-Host "  Este script prepara el PC para compilar y subir los"
Write-Host "  sketches de test/target/ al robot."
Write-Host ""

# ------------------------------------------------------------------
# 1. arduino-cli
# ------------------------------------------------------------------
Titulo "1 de 5 - arduino-cli"

$arduinoCli = Get-Command arduino-cli -ErrorAction SilentlyContinue

if (-not $arduinoCli -and (Test-Path "C:\arduino-cli\arduino-cli.exe")) {
    # Instalado a mano en la ruta que usa el README principal
    $env:Path += ";C:\arduino-cli"
    $arduinoCli = Get-Command arduino-cli -ErrorAction SilentlyContinue
}

if ($arduinoCli) {
    Bien "Ya instalado: $((arduino-cli version) -join '')"
} else {
    Paso "No esta instalado. Instalando con winget..."
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        Malo "No hay winget en este PC."
        Write-Host ""
        Write-Host "  Instala arduino-cli a mano:"
        Write-Host "    1. Descarga el ZIP de Windows 64-bit desde"
        Write-Host "       https://arduino.github.io/arduino-cli/latest/installation/"
        Write-Host "    2. Descomprime arduino-cli.exe en C:\arduino-cli\"
        Write-Host "    3. Anade C:\arduino-cli al PATH"
        Write-Host "    4. Vuelve a ejecutar este script"
        exit 1
    }
    winget install --id ArduinoSA.CLI -e --accept-source-agreements --accept-package-agreements
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [Environment]::GetEnvironmentVariable("Path", "User")
    if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
        Aviso "Instalado, pero no aparece en el PATH de esta ventana."
        Write-Host "  Cierra PowerShell, abrelo de nuevo y vuelve a ejecutar el script."
        exit 1
    }
    Bien "arduino-cli instalado"
}

# ------------------------------------------------------------------
# 2. Indice de placas
# ------------------------------------------------------------------
Titulo "2 de 5 - Indice de placas"

Paso "Anadiendo la URL de las placas ESP32..."
$urlEsp32 = "https://espressif.github.io/arduino-esp32/package_esp32_index.json"

# config init falla si ya existe: no es un error real
try { arduino-cli config init --overwrite | Out-Null } catch { }

try {
    arduino-cli config add board_manager.additional_urls $urlEsp32 2>$null | Out-Null
} catch {
    # Las versiones antiguas usan "set" en vez de "add"
    try { arduino-cli config set board_manager.additional_urls $urlEsp32 | Out-Null } catch { }
}
Bien "URL configurada"

Paso "Actualizando el indice (puede tardar)..."
arduino-cli core update-index
Bien "Indice actualizado"

# ------------------------------------------------------------------
# 3. Soporte de placas
# ------------------------------------------------------------------
Titulo "3 de 5 - Soporte de placas"

$coresInstalados = (arduino-cli core list) -join "`n"

if ($coresInstalados -match "esp32:esp32") {
    Bien "ESP32 ya instalado"
} else {
    Paso "Instalando el soporte del ESP32. Son varios cientos de MB:"
    Paso "es normal que tarde unos minutos."
    arduino-cli core install esp32:esp32
    Bien "ESP32 instalado"
}

if ($coresInstalados -match "arduino:samd") {
    Bien "Arduino MKR ya instalado"
} else {
    Paso "Instalando el soporte del Arduino MKR..."
    arduino-cli core install arduino:samd
    Bien "Arduino MKR instalado"
}

# ------------------------------------------------------------------
# 4. Librerias
# ------------------------------------------------------------------
Titulo "4 de 5 - Librerias"

$libsInstaladas = (arduino-cli lib list) -join "`n"

if ($libsInstaladas -match "(?im)^CAN\s") {
    Bien "Libreria CAN ya instalada"
} else {
    Paso "Instalando la libreria CAN (para la prueba 2, el MKR)..."
    arduino-cli lib install "CAN"
    Bien "Libreria CAN instalada"
}

Write-Host ""
Write-Host "  Nota: las pruebas 1, 3 y 4 solo usan Wire, que viene" -ForegroundColor DarkGray
Write-Host "  incluida con el ESP32. No hace falta instalar nada mas." -ForegroundColor DarkGray

# ------------------------------------------------------------------
# 5. Comprobacion: que los 4 sketches compilen de verdad
# ------------------------------------------------------------------
Titulo "5 de 5 - Comprobando que todo compila"

$raiz = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$fallos = 0

$pruebas = @(
    @{ nombre = "test_encoders"; fqbn = "esp32:esp32:esp32doit-devkit-v1" },
    @{ nombre = "test_steppers"; fqbn = "esp32:esp32:esp32doit-devkit-v1" },
    @{ nombre = "test_imu";      fqbn = "esp32:esp32:esp32doit-devkit-v1" },
    @{ nombre = "test_can_mkr";  fqbn = "arduino:samd:mkrwifi1010"        }
)

foreach ($p in $pruebas) {
    $ruta = Join-Path $raiz "test\target\$($p.nombre)"
    Paso "Compilando $($p.nombre)..."
    if (-not (Test-Path $ruta)) {
        Malo "No se encuentra la carpeta $ruta"
        $fallos++
        continue
    }
    arduino-cli compile --fqbn $p.fqbn $ruta 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) {
        Bien "$($p.nombre) compila"
    } else {
        Malo "$($p.nombre) NO compila"
        Write-Host "     Repite el comando sin silenciar la salida para ver el error:"
        Write-Host "     arduino-cli compile --fqbn $($p.fqbn) $ruta"
        $fallos++
    }
}

# ------------------------------------------------------------------
# Resumen
# ------------------------------------------------------------------
Titulo "Resultado"

if ($fallos -eq 0) {
    Write-Host ""
    Bien "Los 4 sketches compilan. El entorno esta listo."
    Write-Host ""
    Write-Host "  SIGUIENTE PASO: conecta la placa por USB y mira que puerto es:"
    Write-Host ""
    Write-Host "      arduino-cli board list" -ForegroundColor White
    Write-Host ""
    Write-Host "  Si la placa no aparece, casi siempre falta el driver USB:"
    Write-Host "    - ESP32 DevKit V1 -> chip CP2102 (driver de Silicon Labs)"
    Write-Host "                         o CH340 (driver de WCH)"
    Write-Host "    - Arduino MKR     -> no necesita driver en Windows 10/11"
    Write-Host ""
    Write-Host "  Luego, para subir la primera prueba (la mas segura):"
    Write-Host ""
    Write-Host "      arduino-cli upload -p COMx --fqbn esp32:esp32:esp32doit-devkit-v1 test\target\test_encoders" -ForegroundColor White
    Write-Host ""
    Write-Host "  Y abre el Monitor Serie a 115200 baudios."
    Write-Host ""
    Write-Host "  Lee test\target\README.md antes de tocar el robot."
    Write-Host ""
} else {
    Write-Host ""
    Malo "$fallos de 4 sketches no compilan."
    Write-Host ""
    Write-Host "  Revisa los errores de arriba. Lo mas habitual es que falte"
    Write-Host "  algun soporte de placa: vuelve a ejecutar este script."
    Write-Host ""
    exit 1
}
