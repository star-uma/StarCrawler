# compilar_todo.ps1 — compila todas las variantes del firmware contra StarCrawlerHW.
# Uso, desde cualquier carpeta:  ./firmware/compilar_todo.ps1
# Devuelve 1 si alguna falla.

$ErrorActionPreference = "Stop"
$aqui = Split-Path -Parent $MyInvocation.MyCommand.Path
$bibliotecas = Join-Path $aqui "libraries"
$fqbn = "esp32:esp32:esp32doit-devkit-v1"
$variantes = @("starcrawler_esp32", "starcrawler_esp32_basico", "starcrawler_esp32_ros2")

$fallos = 0
foreach ($v in $variantes) {
    Write-Host ("{0,-28}" -f $v) -NoNewline
    $salida = & arduino-cli compile --fqbn $fqbn --libraries $bibliotecas (Join-Path $aqui $v) 2>&1 | Out-String
    if ($LASTEXITCODE -eq 0) {
        $tam = [regex]::Match($salida, "usa (\d+) bytes|uses (\d+) bytes")
        Write-Host ("OK  {0} bytes" -f ($tam.Groups[1].Value + $tam.Groups[2].Value)) -ForegroundColor Green
    } else {
        Write-Host "ERROR" -ForegroundColor Red
        $salida -split "`n" | Where-Object { $_ -match "error" } | Select-Object -First 5 | ForEach-Object { Write-Host "    $_" }
        $fallos++
    }
}

if ($fallos -gt 0) { Write-Host "`n$fallos variante(s) con errores" -ForegroundColor Red; exit 1 }
Write-Host "`nTodas las variantes compilan" -ForegroundColor Green
