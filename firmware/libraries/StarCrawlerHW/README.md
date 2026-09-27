# StarCrawlerHW — código común del firmware

Una sola copia de lo que comparten las variantes de `firmware/`
(`starcrawler_esp32`, `_basico` y `_ros2`). Antes estaba copiado en cada una y
las copias ya habían divergido: la rampa de los steppers solo había llegado a
algunas (issue #10).

| Fichero | Qué es |
|---|---|
| `hw_comun.h` | configuración del hardware: CAN, pines, cadena de elevación, encoders. Lo que salga de la calibración se apunta aquí |
| `control_core.h/.cpp` | lógica pura de control. Sin Arduino: la compilan también los tests de `test/host/` |
| `can_bus.h` + `can_bus_impl.h` | bus CAN, backends MCP2515 y TWAI |
| `steppers.h` + `steppers_impl.h` | pulsos de los DM542 con rampa de aceleración |
| `encoders.h` + `encoders_impl.h` | los 4 AS5600 tras el TCA9548A |

## Por qué hay `_impl.h`

Arduino compila las bibliotecas por separado y **una biblioteca no ve el
`config.h` del sketch**. `can_bus`, `steppers` y `encoders` dependen de él (del
backend CAN, de los pines, de los offsets), así que su implementación no va en
un `.cpp` de la biblioteca: va en un `_impl.h` que incluye el `hw.cpp` de cada
variante, después de su `config.h`:

```cpp
#include "config.h"      // que a su vez incluye <hw_comun.h>

#include <can_bus_impl.h>
#include <steppers_impl.h>
#include <encoders_impl.h>
```

`control_core` no depende de la configuración y es un `.cpp` normal.

## Compilar

La biblioteca vive en el repo, no en la carpeta de bibliotecas de Arduino, así
que hay que decírselo al compilador. Desde `firmware/`:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --libraries libraries starcrawler_esp32_ros2
```

Con el Arduino IDE o la extensión de VS Code, que no aceptan esa opción, se
enlaza una vez en la carpeta de bibliotecas (PowerShell, desde la raíz del
repo):

```powershell
New-Item -ItemType Junction -Path "$env:USERPROFILE\Documents\Arduino\libraries\StarCrawlerHW" -Target "$PWD\firmware\libraries\StarCrawlerHW"
```

## Qué no está aquí

- La app de micro-ROS (`micro_ros_esp32_apps/`) es ESP-IDF, no Arduino, y
  tiene su propio `hw.c`.
- La variante standalone vive en su rama; se pasará a la biblioteca al
  fusionarla.
