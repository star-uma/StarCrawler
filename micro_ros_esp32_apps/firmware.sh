#!/bin/bash
# Compila y flashea la app de micro-ROS con el workspace de micro_ros_setup.
#
#   ./micro_ros_esp32_apps/firmware.sh compilar              # robot real
#   ./micro_ros_esp32_apps/firmware.sh compilar --simulado   # HW_SIMULADO=1
#   ./micro_ros_esp32_apps/firmware.sh flashear /dev/ttyUSB0
#
# El workspace es ~/microros_ws (o MICROROS_WS). Ver starcrawler_app/README.md.
set -eo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="${MICROROS_WS:-$HOME/microros_ws}"
FW="$WS/firmware"
APP="$FW/freertos_apps/apps/starcrawler_app"
BIN="$FW/freertos_apps/microros_esp32_extensions/build/starcrawler_app.bin"

[ -d "$FW/freertos_apps" ] || { echo "No hay firmware en $WS (create_firmware_ws.sh)"; exit 1; }

DISTRO="${ROS_DISTRO:-$(ls /opt/ros | head -n1)}"
source "/opt/ros/$DISTRO/setup.bash"
source "$WS/install/local_setup.bash"
cd "$WS"

case "${1:-}" in
compilar)
    rm -rf "$APP"
    cp -r "$REPO/micro_ros_esp32_apps/starcrawler_app" "$APP"
    if [ "${2:-}" = "--simulado" ]; then
        sed -i 's/^#define HW_SIMULADO 0/#define HW_SIMULADO 1/' "$APP/config.h"
        grep -q '^#define HW_SIMULADO 1' "$APP/config.h"
        echo ">>> HW_SIMULADO=1: planta simulada, no toca ningun pin"
    fi
    # Los mensajes propios van compilados dentro de libmicroros
    if ! diff -rq "$REPO/ros2_ws/src/starcrawler_msgs" "$FW/mcu_ws/starcrawler_msgs" >/dev/null; then
        rm -rf "$FW/mcu_ws/starcrawler_msgs"
        cp -r "$REPO/ros2_ws/src/starcrawler_msgs" "$FW/mcu_ws/"
        echo ">>> starcrawler_msgs actualizado en el firmware"
    fi
    ros2 run micro_ros_setup build_firmware.sh
    echo ">>> $(stat -c %s "$BIN") bytes: $BIN"
    ;;
flashear)
    [ -n "${2:-}" ] || { echo "Falta el puerto: flashear /dev/ttyUSB0"; exit 1; }
    ESPPORT="$2" ros2 run micro_ros_setup flash_firmware.sh
    ;;
*)
    sed -n 2,8p "$0"
    exit 1
    ;;
esac
