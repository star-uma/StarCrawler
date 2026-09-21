# StarCrawler como nodo micro-ROS

El ESP32 deja de ser un esclavo con protocolo serie propio y pasa a ser un
**nodo ROS 2 nativo**, igual que hace Donatello (`star-uma/TFG_MARIA_JOSE`).

> ⚠️ **Nada de esto se ha compilado nunca.** Ver "Estado" al final.

## Qué cambia

| | Antes (`firmware/starcrawler_esp32_ros2`) | Ahora |
|---|---|---|
| Transporte | trama propia con CRC16 por serie | micro-ROS (DDS-XRCE) |
| Traducción | nodo `starcrawler_driver` en el PC | ninguna, el ESP32 publica directo |
| `proto.c` / `protocol.py` | necesarios | desaparecen |
| Conversión de ángulos | en `protocol.py`, en el PC | en `app.c`, en el ESP32 |

### Interfaz ROS

```
Suscripciones
  /cmd_vel           geometry_msgs/Twist
  /crawler/command   starcrawler_msgs/CrawlerCommand

Publicaciones
  /starcrawler/state starcrawler_msgs/RobotState   50 Hz
  /joint_states      sensor_msgs/JointState        50 Hz
```

### Tareas

Mismo reparto que Donatello: lo que no puede tener jitter va al núcleo 1, y
las comunicaciones al 0.

| Tarea | Núcleo | Periodo | Qué hace |
|---|---|---|---|
| `control` | 1 | 10 ms | encoders, lazo de posición, steppers, tramas CAN |
| `micro_ros` | 0 | 20 ms | gira el executor y publica estado |
| `wdt` | 0 | 50 ms | si el executor se cuelga, para y reinicia |

La ISR de los steppers sigue corriendo a 20 kHz por timer hardware, igual que
en la versión Arduino, con su rampa de aceleración.

## Qué se reutiliza sin tocar

`control_core.c` es **C puro sin dependencias** y está cubierto por los tests
de `test/host/` (66/66). Se copia tal cual: toda la lógica de control —
saturación, rampa, límites software, histéresis de posición, tramas RMD —
viene ya probada.

Eso fue una decisión deliberada desde el principio, y es lo que hace que esta
migración sea razonable: lo que se reescribe es el andamiaje, no el control.

## Compilar

Necesitas el sistema de compilación de micro-ROS. Sigue la receta del
laboratorio: <https://github.com/jmgandarias/micro_ros_esp32_apps>

```bash
# 1. Workspace de firmware (una vez)
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# 2. Copiar esta carpeta a las apps del firmware
cp -r micro_ros_esp32_apps/starcrawler_app \
      firmware/freertos_apps/apps/

# 3. Los mensajes propios tienen que estar en el firmware
cp -r ros2_ws/src/starcrawler_msgs \
      firmware/mcu_ws/

# 4. Configurar, compilar y flashear
ros2 run micro_ros_setup configure_firmware.sh starcrawler_app -t serial
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```

Y en el PC de a bordo, el agente:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/starcrawler -b 921600
```

> El paso 3 es el que más se olvida. Sin `starcrawler_msgs` dentro de
> `mcu_ws`, la compilación falla al no encontrar los tipos.

## Versiones

Escrito contra **ESP-IDF v4.4**, que es lo que usa el camino
`freertos esp32` de `micro_ros_setup`. Las APIs sensibles a la versión están
las tres en `hw.c` y marcadas:

| API | v4.4 (este código) | Otras versiones |
|---|---|---|
| CAN | `driver/twai.h` | `driver/can.h` en IDF < 4.2 (lo que usa Donatello) |
| Timer | `driver/timer.h` | `driver/gptimer.h` en IDF 5.x |
| I2C | `driver/i2c.h` | `driver/i2c_master.h` en IDF 5.2+ |

Si el entorno resulta ser otra versión, son esos tres bloques los que hay que
tocar. El resto del código no depende de la versión.

## Qué cambia en el lado del PC

`starcrawler_driver` deja de hacer falta: era el puente que traducía tramas
serie a tópicos. Los demás paquetes (`description`, `teleop`, `msgs`,
`bringup`) siguen igual, porque los tópicos y los tipos no cambian.

El launch de `bringup` habrá que ajustarlo para lanzar el agente de micro-ROS
en vez del nodo driver.

## Estado

**Sin verificar de ninguna manera.** Escrito sin acceso al entorno de
compilación, así que no ha pasado ni por el compilador.

Lo único comprobado:

- `control_core.c` compila como C11 con `-Wall -Wextra` sin avisos
- La estructura sigue la de `control_app/app.c` de Donatello

Lo que hay que esperar en la primera compilación: nombres de API que hayan
cambiado de versión, y los tipos de los mensajes propios. Son errores de
compilación, ruidosos y rápidos de arreglar — no fallos silenciosos.

**Antes de flashearlo al robot**, que funcione con el robot sobre tacos y
habiendo pasado la puesta en marcha de `test/target/`.
