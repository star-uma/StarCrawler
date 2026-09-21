#!/usr/bin/env bash
# =====================================================================
#  start_starcrawler.sh - arranque del robot en el PC de a bordo
# =====================================================================
#
#  Equivalente de executables/start_donatello.sh de Donatello: carga el
#  entorno de ROS y lanza el bringup. Lo llama el servicio de systemd,
#  pero tambien sirve para arrancar a mano.
#
#      ./scripts/start_starcrawler.sh
#
#  Por que un script y no el source metido en el ExecStart del servicio:
#  asi se puede probar el arranque exacto que hara systemd sin habilitar
#  el servicio, que es donde se descubren los fallos de entorno.
#
# =====================================================================

set -uo pipefail

# --- Configuracion ---------------------------------------------------

# Distribucion de ROS 2. Se detecta sola si solo hay una instalada.
ROS_DISTRO_FIJA="${ROS_DISTRO_FIJA:-}"

# Dominio DDS. Dos robots en la misma red con el mismo dominio se ven
# entre ellos y se pisan los topicos: si hay mas de uno en el
# laboratorio, que cada uno tenga el suyo.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

REGISTRO="/tmp/starcrawler_start.log"

# --- Localizar el workspace ------------------------------------------

# Relativo al script, no una ruta absoluta: asi no hay que editarlo
# segun el usuario del PC de a bordo.
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$RAIZ/ros2_ws"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "ERROR: el workspace no esta compilado en $WS" >&2
  echo "       Compila primero:  cd $WS && colcon build --symlink-install" >&2
  exit 1
fi

# --- Cargar ROS -------------------------------------------------------

if [ -n "$ROS_DISTRO_FIJA" ]; then
  DISTRO="$ROS_DISTRO_FIJA"
else
  # shellcheck disable=SC2012
  DISTRO="$(ls -1 /opt/ros 2>/dev/null | head -1)"
fi

if [ -z "${DISTRO:-}" ] || [ ! -f "/opt/ros/$DISTRO/setup.bash" ]; then
  echo "ERROR: no encuentro ninguna instalacion de ROS 2 en /opt/ros" >&2
  echo "       Si hay varias, fija la que toque:" >&2
  echo "         ROS_DISTRO_FIJA=humble $0" >&2
  exit 1
fi

# shellcheck disable=SC1090,SC1091
source "/opt/ros/$DISTRO/setup.bash"
# shellcheck disable=SC1090,SC1091
source "$WS/install/setup.bash"

# --- Esperar al puerto del ESP32 --------------------------------------

# En el arranque del sistema, udev puede tardar en crear el enlace. Si el
# driver arranca antes, falla y systemd lo reintenta, pero se pierden
# segundos y ensucia el log: mejor esperar aqui.
PUERTO="${PUERTO_ESP32:-/dev/starcrawler}"
for _ in $(seq 1 20); do
  [ -e "$PUERTO" ] && break
  sleep 0.5
done

if [ ! -e "$PUERTO" ]; then
  echo "AVISO: $PUERTO no aparece. Arranco igualmente; el driver lo" >&2
  echo "       reintentara. Revisa la regla udev si no llega a conectar." >&2
fi

# --- Arrancar ---------------------------------------------------------

echo "=== StarCrawler $(date '+%Y-%m-%d %H:%M:%S') ==="
echo "ROS $DISTRO | dominio $ROS_DOMAIN_ID | puerto $PUERTO"

exec ros2 launch starcrawler_bringup robot.launch.py \
     port:="$PUERTO" rviz:=false >> "$REGISTRO" 2>&1
