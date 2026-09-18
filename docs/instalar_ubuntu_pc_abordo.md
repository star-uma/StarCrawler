# Instalar Ubuntu en el mini PC de a bordo

Cómo dejar el mini PC con sistema operativo, desde cero. Es el paso **previo**
a [`ros2.md`](ros2.md), que explica la instalación de ROS 2 y del workspace.

Escrito para quien no haya instalado Linux nunca.

---

## Antes: qué versión

Depende de la distribución de ROS 2 que se elija, porque **ROS 2 se distribuye
por versión de Ubuntu** y no se puede mezclar:

| ROS 2 | Ubuntu | Fin de soporte |
|---|---|---|
| **Humble** | 22.04 | mayo 2027 |
| **Jazzy** | 24.04 | mayo 2029 |

La decisión está sin cerrar. El resumen del argumento:

- **Humble** es lo que usa el laboratorio — el TFG de Donatello
  (`star-uma/TFG_MARIA_JOSE`) y el entorno `uma_environment` van sobre Humble,
  igual que la receta de micro-ROS para ESP32 que seguiríamos. Si algo falla,
  hay gente cerca que ya se lo ha encontrado.
- **Jazzy** da dos años más de soporte, pero abre camino en solitario.

Merece la pena preguntar al laboratorio si tienen previsto migrar a Jazzy:
si van a hacerlo, mejor ir directamente allí y ahorrarse la reinstalación.
Sin esa respuesta, **Humble**.

Descarga la **Desktop**, no la Server: hace falta entorno gráfico para RViz.

- Ubuntu 22.04 → <https://releases.ubuntu.com/22.04/>
- Ubuntu 24.04 → <https://releases.ubuntu.com/24.04/>

---

## 1. Preparar el USB

Un pendrive de 8 GB o más. **Se borra entero.**

Desde Windows, con [Rufus](https://rufus.ie): seleccionas el pendrive,
seleccionas la ISO descargada, y Empezar. Si pregunta por el modo de escritura,
elige **imagen DD**.

## 2. Arrancar desde el USB

Conecta al mini PC el pendrive, un teclado, un ratón y un monitor.

Al encender, pulsa repetidamente la tecla de la BIOS. Varía según el
fabricante: suele ser **Supr**, **F2**, **F7** o **F12**. Si no aciertas,
apaga y prueba con otra.

Dentro de la BIOS:

- **Desactiva Secure Boot** — evita problemas con drivers de terceros.
- Pon el USB como primer dispositivo de arranque.

Guarda y sal.

## 3. Instalar

1. Elige **Install Ubuntu**.
2. Teclado: **Español**.
3. Conecta a internet. Mejor por cable que por WiFi durante la instalación.
4. Marca **instalación normal** y **"Instalar software de terceros"** — trae
   drivers de gráficos y WiFi que después cuesta añadir.
5. Disco: **"Borrar disco e instalar Ubuntu"** si el mini PC va a dedicarse
   solo a esto. Borra todo lo que hubiera.
6. Usuario y contraseña. Ponle un nombre de equipo reconocible,
   por ejemplo `starcrawler`.

> ### ⚠️ No cifres el disco
>
> Es la trampa importante para un robot. El cifrado pide contraseña **en cada
> arranque**, antes de cargar el sistema. Con eso, el arranque automático por
> `systemd` que está previsto en `starcrawler_bringup` **no funciona**: el
> robot se queda esperando a que alguien teclee.

Cuando termine, saca el pendrive y reinicia.

## 4. Primeros comandos

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt install -y openssh-server && hostname -I
```

Eso te da la IP del mini PC.

### Trabajar desde tu portátil (recomendado)

Con `openssh-server` instalado puedes usar **VS Code + extensión Remote-SSH**
desde tu portátil: editas en tu máquina, ejecuta la del robot.

Es mejor que desarrollar directamente en el mini PC por dos motivos: no tienes
que colgarle monitor y teclado al robot, y evitas que un experimento rompa la
máquina que tiene que hacerlo funcionar.

## 5. Instalar ROS 2 y el workspace

```bash
git clone -b feature/ros2 https://github.com/star-uma/StarCrawler.git
cd StarCrawler
./scripts/instalar_pc_abordo.sh humble
```

(o `jazzy`). El script instala ROS 2, compila el workspace, te añade al grupo
`dialout` e instala la regla udev del ESP32. El detalle está en
[`ros2.md`](ros2.md).

---

## Cuando esté montado en el robot

Dos cosas de integración eléctrica, ambas en [`ros2.md`](ros2.md) §2:

- **Interruptor propio para el PC.** Cortarle la corriente en caliente una y
  otra vez acaba corrompiendo el sistema de ficheros, y entonces toca repetir
  toda esta instalación.
- **DC-DC regulado** desde la batería: 12 V para un mini PC x86, 5 V y 5 A si
  fuera una Raspberry Pi. La batería del TFG es de Li-ion a medida y no da esas
  tensiones directamente.

Y reserva un puerto USB para el ESP32.
