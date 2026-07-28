# run_tests.ps1 — compila y ejecuta los tests unitarios en el PC.
# Requiere g++ (MinGW) en el PATH.
#
#   1. control_core  (logica de control, firmware unificado/basico)
#   2. gamepad_core  (logica del mando, firmware standalone)

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

# ── 2. gamepad_core (usa el control_core del standalone) ────────────────────
$exe2 = Join-Path $build "test_gamepad_core.exe"
Write-Host "`n[2/2] Compilando tests de gamepad_core..." -ForegroundColor Cyan
g++ -std=c++11 -Wall -Wextra -O2 `
    -o $exe2 `
    (Join-Path $aqui "test_gamepad_core.cpp") `
    (Join-Path $aqui "..\..\firmware\starcrawler_esp32_standalone\gamepad_core.cpp") `
    (Join-Path $aqui "..\..\firmware\starcrawler_esp32_standalone\control_core.cpp")

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
