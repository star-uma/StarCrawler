# run_tests.ps1 — compila y ejecuta los tests unitarios de control_core en el PC.
# Requiere g++ (MinGW) en el PATH.

$ErrorActionPreference = "Stop"
$aqui = Split-Path -Parent $MyInvocation.MyCommand.Path

$build = Join-Path $aqui "build"
if (-not (Test-Path $build)) { New-Item -ItemType Directory $build | Out-Null }

$exe = Join-Path $build "test_control_core.exe"

Write-Host "Compilando tests..." -ForegroundColor Cyan
g++ -std=c++11 -Wall -Wextra -O2 `
    -o $exe `
    (Join-Path $aqui "test_control_core.cpp") `
    (Join-Path $aqui "..\..\firmware\starcrawler_esp32\control_core.cpp")

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR de compilacion" -ForegroundColor Red
    exit 1
}

Write-Host "Ejecutando..." -ForegroundColor Cyan
& $exe
exit $LASTEXITCODE
