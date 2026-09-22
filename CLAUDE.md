# StarCrawler — instrucciones para Claude

Robot de orugas con cuatro brazos articulados. Proyecto de la UMA, sucesor del
robot Horu. Rama de trabajo: **`feature/ros2`**.

**Antes de ponerte a nada, lee [`README-CLAUDE.md`](README-CLAUDE.md).** Tiene
el estado real del proyecto, las limitaciones del entorno y las tareas en
orden.

---

## Lo que hay que tener presente siempre

**Casi nada está verificado.** Los nueve paquetes de `ros2_ws/` no se han
compilado nunca, la app de `micro_ros_esp32_apps/` tampoco, y no hay nada
probado contra el robot. Buena parte de este código se escribió en un ordenador
donde no se podía instalar ROS 2.

Que un fichero exista y esté bien razonado no significa que funcione. Antes de
decir que algo va, comprueba si se ha ejecutado de verdad o solo se ha escrito.
No des por bueno lo que digan los docs sin contrastarlo con el código.

## Cómo trabajar aquí

- **Commits sencillos**: asunto y dos o tres líneas como mucho. **Nunca línea
  de coautor.**
- **El razonamiento va a los `.md` y a las issues, no al código.** Comentarios
  escuetos; si Mario los borra, no los repongas.
- **No commitees notas ni documentos por iniciativa propia.** Déjalos en el
  árbol y que decida él.
- **`main` está congelado**, se arregla cuando se mergee. No lo toques.

## Convenios del proyecto

El orden de los vectores es `{FR, FL, RR, RL}` en todo el proyecto: firmware,
ROS 2, configuración y documentación.

FL y RR van espejadas en los ángulos de encoder. La conversión entre grados de
encoder y radianes de elevación vive en
`ros2_ws/src/starcrawler_common/starcrawler_common/angulos.py` y es la **única
fuente de verdad**: estuvo duplicada en el simulador y en la GUI, y un error de
signo ahí no da un fallo evidente — el robot funciona, pero mueve los brazos al
revés. No la reimplementes en otro sitio.

Las constantes de la cadena de elevación están duplicadas en los `config.h` de
las cuatro variantes de firmware. Un cambio hay que replicarlo en las cuatro
(issue #10). De dónde sale cada número: [`docs/cadena_de_elevacion.md`](docs/cadena_de_elevacion.md).
