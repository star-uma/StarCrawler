#!/usr/bin/env bash
# =====================================================================
#  instalar_entorno.sh - StarCrawler
# =====================================================================
#
#  Deja un PC con Ubuntu listo para compilar y subir los sketches de
#  test/target/ al robot. Pensado para una maquina limpia.
#
#  Es el equivalente de instalar_entorno.ps1, que es el de Windows.
#
#  QUE INSTALA
#    - arduino-cli
#    - El soporte de placa del ESP32       -> pruebas 1, 3 y 4
#    - El soporte de placa del Arduino MKR -> prueba 2
#    - La libreria CAN                     -> prueba 2
#    - Te mete en el grupo "dialout", que hace falta en Linux para
#      poder abrir el puerto USB de la placa
#
#  QUE **NO** INSTALA
#    ROS 2. Eso es otra cosa y va aparte: docs/ros2.md seccion 3.
#    Este script es solo para compilar y flashear el ESP32/MKR.
#
#  COMO SE USA
#    Desde la carpeta del repo:
#
#        ./test/target/instalar_entorno.sh
#
#    Si dice "Permiso denegado", dale permisos de ejecucion primero:
#
#        chmod +x test/target/instalar_entorno.sh
#
#  Tarda un rato la primera vez (el soporte del ESP32 son varios
#  cientos de MB). Se puede volver a ejecutar sin problema.
#
# =====================================================================

set -uo pipefail

VERDE="\033[0;32m"; ROJO="\033[0;31m"; AMAR="\033[0;33m"
CIAN="\033[0;36m";  GRIS="\033[0;90m"; FIN="\033[0m"

titulo() { echo; echo -e "${CIAN}==================================================${FIN}";
           echo -e "${CIAN}  $1${FIN}";
           echo -e "${CIAN}==================================================${FIN}"; }
paso()   { echo "  -> $1"; }
bien()   { echo -e "  ${VERDE}[OK]${FIN} $1"; }
aviso()  { echo -e "  ${AMAR}[!]${FIN}  $1"; }
malo()   { echo -e "  ${ROJO}[ERROR]${FIN} $1"; }

titulo "StarCrawler - instalacion del entorno de pruebas (Ubuntu)"
echo
echo "  Prepara el PC para compilar y subir los sketches de"
echo "  test/target/ al robot."
echo
echo -e "${GRIS}  Esto NO instala ROS 2. Para eso, docs/ros2.md seccion 3.${FIN}"

# ---------------------------------------------------------------------
# 1. arduino-cli
# ---------------------------------------------------------------------
titulo "1 de 6 - arduino-cli"

if command -v arduino-cli >/dev/null 2>&1; then
  bien "Ya instalado: $(arduino-cli version)"
else
  paso "No esta instalado. Descargando el instalador oficial..."
  mkdir -p "$HOME/.local/bin"
  if ! curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
       | BINDIR="$HOME/.local/bin" sh; then
    malo "No se pudo instalar arduino-cli."
    echo "  Revisa la conexion a internet, o instalalo a mano siguiendo"
    echo "  https://arduino.github.io/arduino-cli/latest/installation/"
    exit 1
  fi
  export PATH="$HOME/.local/bin:$PATH"

  # Que siga estando en el PATH en las proximas sesiones
  if ! grep -q '.local/bin' "$HOME/.bashrc" 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    aviso "Anadido ~/.local/bin al PATH en tu ~/.bashrc"
    aviso "En terminales nuevas ya estara; en esta ya lo he puesto."
  fi
  bien "arduino-cli instalado"
fi

# ---------------------------------------------------------------------
# 2. Indice de placas
# ---------------------------------------------------------------------
titulo "2 de 6 - Indice de placas"

URL_ESP32="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

arduino-cli config init --overwrite >/dev/null 2>&1 || true
arduino-cli config add board_manager.additional_urls "$URL_ESP32" >/dev/null 2>&1 \
  || arduino-cli config set board_manager.additional_urls "$URL_ESP32" >/dev/null 2>&1 \
  || true
bien "URL de las placas ESP32 configurada"

paso "Actualizando el indice (puede tardar)..."
if arduino-cli core update-index >/dev/null 2>&1; then
  bien "Indice actualizado"
else
  malo "Fallo al actualizar el indice. Revisa la conexion a internet."
  exit 1
fi

# ---------------------------------------------------------------------
# 3. Soporte de placas
# ---------------------------------------------------------------------
titulo "3 de 6 - Soporte de placas"

CORES="$(arduino-cli core list 2>/dev/null || true)"

if echo "$CORES" | grep -q "esp32:esp32"; then
  bien "ESP32 ya instalado"
else
  paso "Instalando el soporte del ESP32."
  paso "Son varios cientos de MB: es normal que tarde unos minutos."
  if arduino-cli core install esp32:esp32; then
    bien "ESP32 instalado"
  else
    malo "Fallo al instalar el soporte del ESP32."
    exit 1
  fi
fi

if echo "$CORES" | grep -q "arduino:samd"; then
  bien "Arduino MKR ya instalado"
else
  paso "Instalando el soporte del Arduino MKR..."
  if arduino-cli core install arduino:samd; then
    bien "Arduino MKR instalado"
  else
    malo "Fallo al instalar el soporte del Arduino MKR."
    exit 1
  fi
fi

# ---------------------------------------------------------------------
# 4. Librerias
# ---------------------------------------------------------------------
titulo "4 de 6 - Librerias"

if arduino-cli lib list 2>/dev/null | grep -qi "^CAN "; then
  bien "Libreria CAN ya instalada"
else
  paso "Instalando la libreria CAN (para la prueba 2, el MKR)..."
  arduino-cli lib install "CAN" >/dev/null 2>&1 && bien "Libreria CAN instalada" \
    || malo "Fallo al instalar la libreria CAN"
fi

echo
echo -e "${GRIS}  Las pruebas 1, 3 y 4 solo usan Wire, que ya viene con el${FIN}"
echo -e "${GRIS}  soporte del ESP32. No hace falta instalar nada mas.${FIN}"

# ---------------------------------------------------------------------
# 5. Permisos del puerto USB
# ---------------------------------------------------------------------
titulo "5 de 6 - Permisos del puerto USB"

# En Linux, sin estar en el grupo "dialout" no se puede abrir /dev/ttyUSB0
# ni /dev/ttyACM0. Es el fallo numero uno de quien viene de Windows.
if id -nG "$USER" | tr ' ' '\n' | grep -qx "dialout"; then
  bien "Ya estas en el grupo dialout"
else
  paso "Te falta el grupo dialout: sin el no se puede abrir el puerto USB."
  paso "Hace falta sudo para esto."
  if sudo usermod -aG dialout "$USER"; then
    bien "Anadido al grupo dialout"
    echo
    aviso "IMPORTANTE: cierra la sesion y vuelve a entrar (o reinicia)"
    aviso "para que el cambio tenga efecto. Hasta entonces la placa"
    aviso "dara error de permisos al intentar subir el sketch."
  else
    malo "No se pudo anadir al grupo dialout."
    echo "  Hazlo a mano con:  sudo usermod -aG dialout \$USER"
  fi
fi

# ---------------------------------------------------------------------
# 6. Comprobacion: que los 4 sketches compilen de verdad
# ---------------------------------------------------------------------
titulo "6 de 6 - Comprobando que todo compila"

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FALLOS=0

comprobar() { # $1=nombre  $2=fqbn
  local ruta="$RAIZ/test/target/$1"
  paso "Compilando $1..."
  if [ ! -d "$ruta" ]; then
    malo "No se encuentra la carpeta $ruta"
    FALLOS=$((FALLOS+1)); return
  fi
  if arduino-cli compile --fqbn "$2" "$ruta" >/dev/null 2>&1; then
    bien "$1 compila"
  else
    malo "$1 NO compila"
    echo "     Repite el comando para ver el error completo:"
    echo "     arduino-cli compile --fqbn $2 $ruta"
    FALLOS=$((FALLOS+1))
  fi
}

comprobar test_encoders    "esp32:esp32:esp32doit-devkit-v1"
comprobar test_imu         "esp32:esp32:esp32doit-devkit-v1"
comprobar test_steppers_all "esp32:esp32:esp32doit-devkit-v1"
comprobar test_steppers_one "esp32:esp32:esp32doit-devkit-v1"
comprobar test_par_stepper  "esp32:esp32:esp32doit-devkit-v1"
comprobar test_can_esp32    "esp32:esp32:esp32doit-devkit-v1"
comprobar test_can_mkr      "arduino:samd:mkrwifi1010"
comprobar test_motor_diag   "arduino:samd:mkrwifi1010"

# ---------------------------------------------------------------------
# Resumen
# ---------------------------------------------------------------------
titulo "Resultado"

if [ "$FALLOS" -eq 0 ]; then
  echo
  bien "Los 8 sketches compilan. El entorno esta listo."
  echo
  echo "  SIGUIENTE PASO: conecta la placa por USB y mira que puerto es:"
  echo
  echo "      arduino-cli board list"
  echo
  echo "  En Linux suele salir como /dev/ttyUSB0 (ESP32, chip CP2102 o"
  echo "  CH340) o /dev/ttyACM0 (Arduino MKR). Los drivers ya vienen en"
  echo "  el kernel: no hay que instalar nada, pero si hace falta estar"
  echo "  en el grupo dialout (paso 5)."
  echo
  echo "  Para subir la primera prueba, que es la mas segura:"
  echo
  echo "      arduino-cli upload -p /dev/ttyUSB0 \\"
  echo "          --fqbn esp32:esp32:esp32doit-devkit-v1 test/target/test_encoders"
  echo
  echo "  Y para ver la salida:"
  echo
  echo "      arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200"
  echo
  echo "  Lee test/target/README.md antes de tocar el robot."
  echo
else
  echo
  malo "$FALLOS de 8 sketches no compilan."
  echo
  echo "  Revisa los errores de arriba. Lo mas habitual es que falte"
  echo "  algun soporte de placa: vuelve a ejecutar este script."
  echo
  exit 1
fi
