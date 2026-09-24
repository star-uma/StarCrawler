#!/usr/bin/env bash
# =====================================================================
#  instalar_pc_abordo.sh - StarCrawler
# =====================================================================
#
#  Deja el mini PC de a bordo listo para mover el robot: instala ROS 2,
#  compila el workspace y configura el acceso al ESP32.
#
#  Pensado para una maquina limpia. No hace falta saber ROS para usarlo.
#
#  COMO SE USA
#      ./scripts/instalar_pc_abordo.sh            # ROS 2 Humble (por defecto)
#      ./scripts/instalar_pc_abordo.sh jazzy      # ROS 2 Jazzy
#
#  QUE DISTRIBUCION ELEGIR
#      humble -> Ubuntu 22.04. Es la del laboratorio: el TFG de Donatello
#                y el entorno uma_environment van sobre Humble. Si vais a
#                seguir sus instrucciones o reutilizar su codigo, esta es
#                la que reduce fricciones. Soporte hasta mayo de 2027.
#      jazzy  -> Ubuntu 24.04. Soporte hasta 2029, pero os separa del
#                ecosistema del laboratorio.
#
#  QUE **NO** HACE
#      - No instala el entorno de micro-ROS. Eso es aparte y solo hace
#        falta si se migra el firmware del ESP32 a micro-ROS.
#      - No habilita el arranque automatico del robot. Se pregunta al
#        final y por defecto NO se activa: con el servicio habilitado el
#        robot queda operativo nada mas dar tension.
#
#  Se puede volver a ejecutar sin problema.
#
# =====================================================================

set -uo pipefail

DISTRO="${1:-humble}"

VERDE="\033[0;32m"; ROJO="\033[0;31m"; AMAR="\033[0;33m"
CIAN="\033[0;36m";  GRIS="\033[0;90m"; FIN="\033[0m"

titulo() { echo; echo -e "${CIAN}==================================================${FIN}";
           echo -e "${CIAN}  $1${FIN}";
           echo -e "${CIAN}==================================================${FIN}"; }
paso()   { echo "  -> $1"; }
bien()   { echo -e "  ${VERDE}[OK]${FIN} $1"; }
aviso()  { echo -e "  ${AMAR}[!]${FIN}  $1"; }
malo()   { echo -e "  ${ROJO}[ERROR]${FIN} $1"; }

# ---------------------------------------------------------------------
# 0. Comprobaciones previas
# ---------------------------------------------------------------------
titulo "StarCrawler - instalacion del PC de a bordo"

case "$DISTRO" in
  humble) UBUNTU_OK="22.04"; NOMBRE_OK="jammy" ;;
  jazzy)  UBUNTU_OK="24.04"; NOMBRE_OK="noble" ;;
  *) malo "Distribucion no reconocida: '$DISTRO'"
     echo "  Usa:  ./scripts/instalar_pc_abordo.sh humble"
     echo "    o:  ./scripts/instalar_pc_abordo.sh jazzy"
     exit 1 ;;
esac

if [ ! -f /etc/os-release ]; then
  malo "Esto no parece un Ubuntu. El script es solo para Ubuntu."
  exit 1
fi
. /etc/os-release

echo
echo "  ROS 2 a instalar : $DISTRO"
echo "  Ubuntu esperado  : $UBUNTU_OK ($NOMBRE_OK)"
echo "  Ubuntu detectado : ${VERSION_ID:-desconocido} (${UBUNTU_CODENAME:-?})"
echo

if [ "${VERSION_ID:-}" != "$UBUNTU_OK" ]; then
  aviso "La version de Ubuntu no coincide con la que espera $DISTRO."
  aviso "ROS 2 se distribuye por version de Ubuntu: si no coinciden,"
  aviso "los paquetes no existen y la instalacion fallara."
  echo
  echo "  Opciones:"
  echo "    - Instalar Ubuntu $UBUNTU_OK en esta maquina, o"
  echo "    - Lanzar el script con la distribucion que toque a tu Ubuntu"
  echo
  read -r -p "  Continuar de todas formas? (escribe SI) " r
  [ "$r" = "SI" ] || { echo "  Cancelado."; exit 1; }
fi

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$RAIZ/ros2_ws"

if [ ! -d "$WS/src" ]; then
  malo "No encuentro el workspace en $WS"
  echo "  Este script hay que ejecutarlo desde el repo, y en la rama"
  echo "  feature/ros2 (o una que salga de ella), que es donde vive"
  echo "  ros2_ws/. Comprueba con:  git branch --show-current"
  exit 1
fi
bien "Workspace encontrado en $WS"

# ---------------------------------------------------------------------
# 1. Locale
# ---------------------------------------------------------------------
titulo "1 de 6 - Idioma del sistema (UTF-8)"

paso "ROS 2 necesita un locale UTF-8..."
sudo apt-get update -qq
sudo apt-get install -y -qq locales >/dev/null
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
bien "Locale configurado"

# ---------------------------------------------------------------------
# 2. Repositorio de ROS 2
# ---------------------------------------------------------------------
titulo "2 de 6 - Repositorio de ROS 2"

if [ -f /etc/apt/sources.list.d/ros2.list ] || [ -f /etc/apt/sources.list.d/ros2-latest.list ]; then
  bien "El repositorio de ROS 2 ya esta dado de alta"
else
  paso "Anadiendo el repositorio oficial..."
  sudo apt-get install -y -qq software-properties-common curl >/dev/null
  sudo add-apt-repository -y universe >/dev/null 2>&1
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
       | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  bien "Repositorio anadido"
fi

paso "Actualizando la lista de paquetes..."
sudo apt-get update -qq

# ---------------------------------------------------------------------
# 3. ROS 2 y paquetes de StarCrawler
# ---------------------------------------------------------------------
titulo "3 de 6 - ROS 2 $DISTRO"

if [ -d "/opt/ros/$DISTRO" ]; then
  bien "ROS 2 $DISTRO ya esta instalado"
else
  paso "Instalando ROS 2 $DISTRO. Esto tarda bastante (varios GB)."
  if ! sudo apt-get install -y "ros-$DISTRO-desktop" ros-dev-tools; then
    malo "Fallo al instalar ROS 2 $DISTRO."
    echo "  Lo mas habitual: la version de Ubuntu no corresponde."
    echo "  Referencia: https://docs.ros.org/en/$DISTRO/Installation.html"
    exit 1
  fi
  bien "ROS 2 $DISTRO instalado"
fi

paso "Instalando los paquetes que usa StarCrawler..."
sudo apt-get install -y \
    "ros-$DISTRO-joy" \
    "ros-$DISTRO-xacro" \
    "ros-$DISTRO-robot-state-publisher" \
    "ros-$DISTRO-joint-state-publisher-gui" \
    "ros-$DISTRO-rviz2" \
    "ros-$DISTRO-diagnostic-updater" \
    python3-serial
bien "Paquetes instalados"

if ! grep -q "source /opt/ros/$DISTRO/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  echo "source /opt/ros/$DISTRO/setup.bash" >> "$HOME/.bashrc"
  bien "ROS se cargara solo en cada terminal nueva"
else
  bien "ROS ya se cargaba en el .bashrc"
fi

# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"

# ---------------------------------------------------------------------
# 4. Compilar el workspace
# ---------------------------------------------------------------------
titulo "4 de 6 - Compilando el workspace de StarCrawler"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  paso "Inicializando rosdep (solo la primera vez en esta maquina)..."
  sudo rosdep init >/dev/null 2>&1 || true
fi
paso "Actualizando rosdep..."
rosdep update >/dev/null 2>&1 || aviso "rosdep update dio error, sigo igualmente"

paso "Resolviendo dependencias de los paquetes..."
( cd "$WS" && rosdep install --from-paths src --ignore-src -y ) \
  || aviso "rosdep no pudo resolver todo; si falla la compilacion, mira aqui"

paso "Compilando (colcon build). Tarda unos minutos..."
if ( cd "$WS" && colcon build --symlink-install ); then
  bien "Workspace compilado"
else
  malo "La compilacion ha fallado."
  echo "  Vuelve a lanzarla a mano para ver el error completo:"
  echo "      cd $WS && colcon build --symlink-install"
  exit 1
fi

if ! grep -q "$WS/install/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  echo "source $WS/install/setup.bash" >> "$HOME/.bashrc"
  bien "El workspace se cargara solo en cada terminal nueva"
fi

# ---------------------------------------------------------------------
# 5. Acceso al ESP32 por USB
# ---------------------------------------------------------------------
titulo "5 de 6 - Acceso al ESP32 por USB"

# Sin el grupo dialout no se puede abrir /dev/ttyUSB0: el nodo driver
# fallaria con un error de permisos poco explicativo.
if id -nG "$USER" | tr ' ' '\n' | grep -qx "dialout"; then
  bien "Ya estas en el grupo dialout"
else
  paso "Anadiendote al grupo dialout (hace falta para abrir el puerto)..."
  sudo usermod -aG dialout "$USER" && bien "Anadido al grupo dialout"
  aviso "Cierra la sesion y vuelve a entrar para que tenga efecto."
fi

REGLA="$WS/src/starcrawler_bringup/udev/99-starcrawler.rules"
if [ -f "$REGLA" ]; then
  paso "Instalando la regla udev (nombre fijo para el puerto del ESP32)..."
  sudo cp "$REGLA" /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  bien "Regla udev instalada"
  aviso "Revisa la regla: trae comentadas las lineas segun el chip USB"
  aviso "de tu placa (CP2102 o CH340). Hay que descomentar la que toque:"
  aviso "  /etc/udev/rules.d/99-starcrawler.rules"
else
  aviso "No encuentro la regla udev en el workspace, me la salto"
fi

# ---------------------------------------------------------------------
# 6. Arranque automatico (opcional)
# ---------------------------------------------------------------------
titulo "6 de 6 - Arranque automatico (opcional)"

echo
echo "  Se puede configurar el robot para que arranque solo al encender"
echo "  el PC, como hace Donatello con su NUC."
echo
aviso "NO lo actives todavia."
echo "  Con el servicio habilitado el robot queda operativo nada mas dar"
echo "  tension. Eso solo tiene sentido cuando el mapeo del mando este"
echo "  verificado y el robot calibrado."
echo
echo "  Cuando llegue el momento, el fichero esta en:"
echo "      $WS/src/starcrawler_bringup/systemd/starcrawler.service"
echo "  Hay que ajustar el usuario y las rutas, y despues:"
echo "      sudo cp ... /etc/systemd/system/"
echo "      sudo systemctl daemon-reload"
echo "      sudo systemctl enable --now starcrawler"
echo

# ---------------------------------------------------------------------
# Resumen
# ---------------------------------------------------------------------
titulo "Listo"

echo
bien "ROS 2 $DISTRO instalado y el workspace compilado."
echo
echo "  ABRE UNA TERMINAL NUEVA (para que se cargue todo) y prueba"
echo "  primero SIN el robot, con el simulador que trae el driver:"
echo
echo "      ros2 launch starcrawler_bringup robot.launch.py simulate:=true rviz:=true"
echo
echo "  Deberias ver el robot en RViz. Para moverlo sin mando:"
echo
echo "      ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \\"
echo "        '{linear: {x: 0.01}, angular: {z: 0.0}}'"
echo
echo -e "${GRIS}  Todo el detalle esta en docs/ros2.md.${FIN}"
echo -e "${GRIS}  Con el robot delante, antes lee test/target/README.md.${FIN}"
echo
