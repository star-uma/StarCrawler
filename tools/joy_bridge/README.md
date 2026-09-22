# Puente del mando: Windows → WSL

WSL2 no ve el mando: no tiene pila Bluetooth y su kernel de serie no trae
drivers HID para pasarlo por USB con `usbipd`. Este puente lo lee en Windows
con pygame (como hacia `control/StarCrawlerXbox.py` en la v1) y lo envia por
UDP a `joy_udp_node`, que lo publica en `/joy`.

El puente entrega ejes y botones **en el mismo orden y signo que el nodo `joy`
de Linux con un DualShock 4** (driver hid-sony), asi que `ds4.yaml` y el teleop
no cambian, y en el PC de a bordo todo funciona igual con el `joy` nativo.

## Uso

1. Emparejar el DS4 con Windows: mantener **Share + PS** hasta que la barra
   parpadee rapido → Bluetooth → Agregar dispositivo → "Wireless Controller".
2. En el WSL, lanzar con el mando por UDP:
   ```bash
   ros2 launch starcrawler_bringup robot.launch.py sim:=true gui:=true joy_udp:=true
   ```
3. En Windows (Python 3.12, que es el que tiene pygame):
   ```powershell
   py -3.12 tools\joy_bridge\joy_bridge.py
   ```
   La IP del WSL la averigua solo (`wsl hostname -I`); si falla, `--ip`.

El puente detecta si SDL ha abierto el mando por HIDAPI ("PS4 Controller",
16 botones) o por DirectInput ("Wireless Controller", 14 botones + hat) y usa
la tabla que toca. Si el mando no responde como se espera,
`py -3.12 joy_bridge.py --probar` muestra los indices crudos de pygame para
ajustar `DISPOSICIONES`.

Verificado (22-09-2026) con un DS4 por Bluetooth y el robot simulado: stick
izquierdo arriba → `linear.x` +máx y la odometria avanza; stick derecho a la
derecha → `angular.z` −máx; X → boton 0. Hay que lanzarlo desde una terminal
de la sesion de escritorio: desde un proceso sin sesion interactiva SDL no
recibe eventos del mando.

Solo para el banco de pruebas en Windows. En el robot no hace falta.
