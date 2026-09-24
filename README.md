# StarCrawler — pruebas con el robot delante

Esta rama sirve para **probar cada componente del robot por separado**, antes
de juntarlo todo: encoders, IMU, motores de tracción (CAN) y motores de
elevación. El trabajo principal del proyecto (ROS 2, micro-ROS, simulador)
está en la rama `feature/ros2`.

**Todo está en [`test/target/README.md`](test/target/README.md)**: qué prueba
hacer, en qué orden, cómo subir cada sketch y qué apuntar.

## Antes de encender nada

1. Las pruebas van **en orden**, de menos a más riesgo.
2. **Robot sobre tacos** en cuanto algo se mueva.
3. **Nunca dos placas mandando en el bus CAN** a la vez.

Preparar el PC la primera vez (en Linux, `instalar_entorno.sh`):

```powershell
./test/target/instalar_entorno.ps1
```

## Qué hay

| Fichero | Para qué |
|---|---|
| [`test/target/README.md`](test/target/README.md) | Las pruebas, paso a paso |
| [`test/target/GUIA_SESION.md`](test/target/GUIA_SESION.md) | El resumen para tener al lado durante la sesión |
| [`test/target/PROMPT_CLAUDE.md`](test/target/PROMPT_CLAUDE.md) | Para abrir Claude en esta rama con el contexto |
| [`docs/cadena_de_elevacion.md`](docs/cadena_de_elevacion.md) | De dónde sale cada constante de los paso a paso |
| [`docs/arquitectura_esp32_unificada.md`](docs/arquitectura_esp32_unificada.md) | El cableado del ESP32 |
| `firmware/` | Los firmwares de Arduino que se flashean después de las pruebas |

Lo que contaba antes este README (la arquitectura v1 con el MKR y el protocolo
UDP) está en `docs/v1_referencia.md` de `feature/ros2`, y en `main`.
