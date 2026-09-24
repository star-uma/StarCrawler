# StarCrawler como nodo micro-ROS

El ESP32 deja de ser un esclavo con protocolo serie propio y pasa a ser un
**nodo ROS 2 nativo**, igual que hace Donatello (`star-uma/TFG_MARIA_JOSE`).

> ⚠️ **Compila, pero no se ha probado en hardware.** Ver "Estado" al final.

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

Publicaciones (best-effort)
  /starcrawler/state starcrawler_msgs/RobotState   50 Hz (49,2 medidos a 921600)
  /joint_states      sensor_msgs/JointState        50 Hz (49,3 medidos a 921600)
```

### Tareas

Mismo reparto que Donatello: lo que no puede tener jitter va al núcleo 1, y
las comunicaciones al 0.

| Tarea | Núcleo | Periodo | Qué hace |
|---|---|---|---|
| `control` | 1 | 10 ms | encoders, lazo de posición, steppers, tramas CAN |
| `micro_ros` | 0 | 20 ms | gira el executor y publica estado |
| `wdt` | 0 | 50 ms | si el executor se cuelga, para y reinicia |

Además, la tarea `micro_ros` hace `rmw_uros_ping_agent` cada segundo: el
cliente XRCE no reenvía `CREATE_CLIENT` por sí solo, así que si el agente se
reinicia (relanzar el launch, reiniciar el PC) la placa se quedaría muda hasta
un apagado. Tras 5 s sin respuesta: parada segura y reinicio. Medido: el
agente relanzado recupera la sesión en ~5 s.

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

Y en el PC de a bordo, el agente. No es un paquete de Humble ni una clave de
rosdep: se construye una vez con `micro_ros_setup` dentro del mismo workspace:

```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/starcrawler -b 921600
```

> El transporte de `freertos_apps` abre el UART a 115200 (literal en
> `microros_transports.c`, sin opción de menuconfig). A esa velocidad no caben
> los dos tópicos a 50 Hz (~270 B por ciclo son ~23 ms de cable; medidos 35 Hz).
> Por eso `app.c` registra su propio transporte, que envuelve al de
> `freertos_apps` y sube el baudio a `SERIE_BAUDIOS` (921600) al abrir. El
> agente y `micro_ros_baud` del launch deben ir al mismo valor.

## Versiones

`micro_ros_setup` (rama humble, ruta `freertos esp32`) construye con
**ESP-IDF v4.1**, no con la 4.4 como se suponía al escribir esto. El código
sigue usando los nombres de la 4.4 y las diferencias se traducen en un único
sitio, `idf_compat.h`:

| Qué | 4.4 (nombres del código) | 4.1 (lo que hay) |
|---|---|---|
| CAN | `driver/twai.h`, `twai_*` | `driver/can.h`, `can_*` — mismo driver, nombre antiguo |
| Espera en µs | `esp_rom_delay_us` (`esp_rom_sys.h`) | `ets_delay_us` (`esp32/rom/ets_sys.h`) |
| I2C | `i2c_master_write_to_device`, `i2c_master_write_read_device` | no existen: se implementan sobre la API de comandos |
| Timer | `timer_isr_callback_add` | existe también en 4.1 |

Si el laboratorio pasa a una IDF más nueva (≥ 4.4), `idf_compat.h` se vuelve
transparente. En IDF 5.x cambian además el timer (`gptimer`) y el I2C
(`i2c_master`), y eso sí habría que portarlo.

## Qué cambia en el lado del PC

`starcrawler_driver` deja de hacer falta: era el puente que traducía tramas
serie a tópicos. Los demás paquetes (`description`, `teleop`, `msgs`,
`bringup`) siguen igual, porque los tópicos y los tipos no cambian.

El launch de `bringup` habrá que ajustarlo para lanzar el agente de micro-ROS
en vez del nodo driver.

## Estado

**Compila** (22-09-2026) con la cadena real: `micro_ros_setup` humble →
ESP-IDF v4.1 → `starcrawler_app.bin` de 459 KB, sin errores ni avisos en los
ficheros de la app. Lo que hubo que tocar en la primera compilación:

- Las APIs de ESP-IDF que no existen en la 4.1 (`idf_compat.h`, ver arriba).
- `app.c` trataba como secuencias los campos que en el `.msg` son arrays fijos
  (`float64[4]`, `int8[4]`, `bool[4]`): en C son arrays planos dentro del
  struct, sin `.data/.size/.capacity` ni memoria que reservar. Solo las
  secuencias de `JointState` (`name`, `position`) necesitan buffers.
- Faltaba `rosidl_runtime_c/string_functions.h`.

**Probado con un ESP32 sin nada conectado** (22-09-2026): el agente establece
una única sesión, el nodo `starcrawler_esp32` publica ambos tópicos y el estado
es el correcto para una placa pelada (`encoder_ok` ×4 false, `can_ok` false,
`safety_active` true, `error_bits` 111 = encoders + CAN + watchdog).

Tasas medidas: con los publishers fiables por defecto a 115200 llegaban 17 Hz
de estado y 12 Hz de `joint_states` (el stream fiable descartaba); en
best-effort, 35 Hz las dos, que es el techo físico de 115200 (~270 B por ciclo
son ~23 ms de cable). Con el transporte a 921600, **49,2 y 49,3 Hz** medidos a
la vez durante 30 s en régimen. En esa ventana hubo un hueco aislado de ~3 s en
ambos tópicos, sin determinar si viene del firmware o del camino
USB→usbipd→WSL del banco; comprobar en el PC de a bordo.

**Hora**: el ESP32 sincroniza con el agente (`rmw_uros_sync_session`) al
arrancar y en cada ping, y sella `/starcrawler/state` y `/joint_states` con
`rmw_uros_epoch_nanos()`. Sin sello (0), `robot_state_publisher` descartaba
todos los `/joint_states`: no habia TF de las orugas y RViz pintaba el modelo
en rojo (la web 3D no lo nota porque no usa TF). Medido (24-09): sello a
[-78, +8] ms de la hora del PC, recolocado en <1 s tras un salto del reloj.

CAN, steppers y encoders siguen **sin verificar**: eso solo se ve con el robot
sobre tacos.

**Antes de flashearlo al robot**, que funcione con el robot sobre tacos y
habiendo pasado la puesta en marcha de `test/target/`.
