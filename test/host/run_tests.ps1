# run_tests.ps1 — compila y ejecuta los tests unitarios en el PC.
# Requiere g++ (MinGW) en el PATH.
#
#   1. control_core — logica de control del firmware
#   2. proto        — protocolo serie ROS 2 (contrato con protocol.py)

$ErrorActionPreference = "Stop"
$aqui = Split-Path -Parent $MyInvocation.MyCommand.Path

$build = Join-Path $aqui "build"
if (-not (Test-Path $build)) { New-Item -ItemType Directory $build | Out-Null }

$fallos = 0

# ── 1. control_core ─────────────────────────────────────────────────────────
$exe1 = Join-Path $build "test_control_core.exe"
Write-Host "`n[1/2] Compilando tests de control_core..." -ForegroundColor Cyan
g++ -std=c++11 -Wall -Wextra -O2 `
    -o $exe1 `
    (Join-Path $aqui "test_control_core.cpp") `
    (Join-Path $aqui "..\..\firmware\starcrawler_esp32\control_core.cpp")

if ($LASTEXITCODE -ne 0) { Write-Host "ERROR de compilacion" -ForegroundColor Red; exit 1 }
& $exe1
if ($LASTEXITCODE -ne 0) { $fallos++ }

# ── 2. protocolo serie ROS 2 ────────────────────────────────────────────────
$exe2 = Join-Path $build "test_proto.exe"
Write-Host "`n[2/2] Compilando tests del protocolo serie..." -ForegroundColor Cyan
g++ -std=c++11 -Wall -Wextra -O2 `
    -o $exe2 `
    (Join-Path $aqui "test_proto.cpp") `
    (Join-Path $aqui "..\..\firmware\starcrawler_esp32_ros2\proto.cpp")

if ($LASTEXITCODE -ne 0) { Write-Host "ERROR de compilacion" -ForegroundColor Red; exit 1 }
& $exe2
if ($LASTEXITCODE -ne 0) { $fallos++ }

# ── Resumen ──────────────────────────────────────────────────────────────────
if ($fallos -gt 0) {
    Write-Host "`nHay suites con fallos." -ForegroundColor Red
    exit 1
}
Write-Host "`nTodas las suites OK." -ForegroundColor Green
exit 0
