## 0. Resumen

Propongo un nodo nuevo, `mundo_node`, en el paquete `starcrawler_sim`. Es el único nodo que conoce el terreno. Escucha lo que publica el robot, venga de starcrawler_sim o del ESP32 en HW_SIMULADO. Integra la pose verdadera teniendo en cuenta el terreno y el bloqueo, y la mete en TF como `map -> odom`, que es el patrón de localización de REP-105. Además publica una IMU simulada en `/imu/data`, la pose verdadera en `/mundo/verdad` y la geometría del mundo.

No escribe nunca al robot ni a `/joint_states`. La odometría, `chasis_node`, `robot_state_publisher`, los muxes y el ESP32 se quedan exactamente como están. Con `mundo:=''`, que es el valor por defecto, el grafo es el de hoy, y el del robot real también.

## 1. Principios

1. **El mundo es un observador que va detrás de `/starcrawler/state`.** Es la única forma de que funcione igual con `sim:=true` que con el ESP32 en HW_SIMULADO. El micro no sabe nada del terreno y no se toca.
2. **Lo que el robot estima de sí mismo sigue saliendo de los mismos nodos que en el robot real:** la odometría 2D y el chasis calculado en llano. La diferencia entre esa estimación y la verdad es justo lo que va en `map -> odom`. REP-105 permite que `map -> odom` salte (por ejemplo, al recolocar el robot), mientras `odom -> base_footprint` sigue siendo continuo.
3. **No hay realimentación del terreno al robot.**
   - El terreno no frena los RMD ni hace perder pasos a los steppers.
   - Si las orugas se bloquean contra un canto, giran y patinan: la odometría sigue avanzando y la verdad no. Así es como se ve el bloqueo.
   - Los brazos llegan siempre a su ángulo; es el chasis el que se acomoda.
   - Es un modelo cuasiestático. Basta para ensayar a 0,053 m/s (40 dps x 0,0764 m) con los brazos a 4,69 grados/s.

## 2. Opciones que he comparado

- **A. `mundo_node` publica `map -> odom`: la elegida.**
  - No toca nada de lo que ya existe y funciona igual con el simulador que con el ESP32.
  - La odometría sigue funcionando y fallando como en el robot real, y el operador ve cuánto miente.
  - Es un añadido que se puede quitar: sin mundo no hay marco `map`.
- **B. Una TF propia, un robot fantasma con `frame_prefix`: descartada.**
  - Hace falta un segundo `robot_state_publisher` y dos modelos en RViz y en la web.
  - La IMU y la verdad no se verían sobre el robot principal.
  - Si algún día interesa, puede quedar como vista opcional del fantasma de la odometría.
- **C. El mundo sustituye a la odometría y publica él `odom -> base_footprint`: descartada.**
  - Con `sim:=true` la odometría sería perfecta, algo que el robot real no tendrá nunca.
  - Con el ESP32 habría que apagar `odometry_node`, justo cuando lo que se quiere probar es la cadena real.
  - Chocaría con un futuro EKF, que también querrá publicar `odom -> base_footprint`.
- **D. Meter el terreno en `sim_node`: descartada.**
  - No sirve para el ESP32 en HW_SIMULADO.
  - Rompe la regla de que `sim_core` imita al firmware ciclo a ciclo.

## 3. Árbol de TF, fórmula y sellos

```
map --[mundo_node, 50 Hz]--> odom --[odometry_node]--> base_footprint --[RSP con las juntas de chasis_node]--> base_link --> crawler_*_link, imu_link
```

**Fórmula.**
```
T_map_odom(t) = T_verdad(base_link en map, t) * inversa(T_est(t))
```
`T_est` es `lookup_transform('odom', 'base_link', Time())`: odometría más las juntas de chasis_node, sea quien sea el que las estime. `t` es el sello de esa consulta. `T_verdad(t)` se interpola en un historial de 2 s de la propia verdad; eso absorbe la dispersión de sellos del ESP32, medida en [-76, +20] ms.

**Resultado.** `map -> base_link` es la verdad en cada instante, aunque cambie el estimador (una IMU en chasis_node, un EKF).

**Propiedades de `map -> odom`:**
- En llano, sin patinar y con los factores a 1,0, es la identidad salvo el desfase entre temporizadores (menos de 1 mm). Esa es la comprobación principal.
- Si el robot se bloquea, crece la traslación.
- En una rampa crece z y, en la fase 1, lleva también el cabeceo que el robot no conoce. Con la IMU en chasis_node (fase 2) queda en x, y, z y guiñada.

**Regla de sellos.**
- Solo se publica si `t` es estrictamente mayor que el último sello publicado. Si no, tf2 imprime TF_REPEATED_DATA en cada oyente, RViz incluido.
- Si `now - t > 0,5 s`, no se publica y se avisa con WARN: la estimación está parada (por ejemplo, con `odom:=false` o con chasis_node caído).
- Va por `/tf`, nunca por `/tf_static`.

**Qué pasa en cada modo:**
- **Robot real:** nadie publica `map -> odom`. `odometry_node` publica `odom -> base_footprint` y chasis_node las juntas en llano. `/imu/data` hoy no existe; será de la IMU real cuando la haya. RViz usa `plano.rviz` con marco fijo `odom`.
- **`sim:=true`:** igual que el robot real.
- **`sim:=true mundo:=X`:** `mundo_node` publica `map -> odom` y `/imu/data`. Lo demás no cambia. RViz usa `mundo.rviz` con marco fijo `map`.
- **micro-ROS, ESP32 en HW_SIMULADO, `mundo:=X`:** igual que el caso anterior.
- **Robot real con `mundo:=X`:** `mundo_node` no publica nada, lo dice con ERROR, y RViz avisa de que no existe `map`.
- **`simulate:=true mundo:=X`:** tampoco publica nada, porque el simulador serie no pone el bit 7.

## 4. Interfaz de `mundo_node`

**Ficheros:**
- En `starcrawler_sim/starcrawler_sim/`:
  - `mundo_core.py`, puro: YAML a piezas y a marcadores, historial y corrección.
  - `terreno_core.py`, puro: el contacto y el bloqueo, que diseña el frente de física.
  - `mundo_node.py`.
- `starcrawler_sim/mundos/*.yaml`.
- Tests en `starcrawler_sim/test/`.
- `mundo.rviz` en `starcrawler_description/rviz/`, junto a `plano.rviz`.

**Dependencias nuevas de `starcrawler_sim`:** tf2_ros, nav_msgs, visualization_msgs, diagnostic_msgs, std_srvs, python3-yaml y starcrawler_odometry. De este último solo se importan `chasis_core.geometria_desde_urdf`, `pose_chasis` y `odometry_core.velocidades_del_robot`, para no copiar ni geometría ni cinemática.

**Suscripciones:**
- `/starcrawler/state`, RobotState, `qos_profile_sensor_data` (best effort, como la odometría y la GUI). Se usan `crawler_angle`, `encoder_ok`, `track_speed_*` y `error_bits`.
- `/robot_description`, String, transient local con depth 1: la geometría sale del URDF, como en chasis_node.
- `/tf` y `/tf_static`, con un Buffer de 2 s y un TransformListener.
- `/initialpose`, PoseWithCovarianceStamped, reliable con depth 1. Es la herramienta 2D Pose Estimate de RViz: exige `frame_id == map`, coloca (x, y, guiñada) y deja caer el robot sobre el terreno.

**Publicaciones:**
- `map -> odom` por TransformBroadcaster, con la QoS estándar de `/tf`. 50 Hz, sellado con `t`.
- `/mundo/verdad`, nav_msgs/Odometry. Reliable, volatile, depth 10, 50 Hz. `frame_id` map y `child_frame_id` base_link. Lleva la pose 6D verdadera; el twist es la v real a lo largo del suelo y la w real.
- `/imu/data`, sensor_msgs/Imu. Reliable, depth 10, así que casa también con suscriptores best effort. 50 Hz, `frame_id` imu_link.
- `/mundo/marcadores`, MarkerArray. Reliable, transient local, depth 1. Se publica al arrancar y al recargar (un DELETEALL y luego las piezas), en map.
- `/mundo/contactos`, MarkerArray, volatile, 10 Hz. Muestra los puntos de apoyo de cada banda, en rojo si está bloqueada.
- `/mundo/estado`, diagnostic_msgs/DiagnosticStatus, 10 Hz.
  - Nivel: OK, WARN si está bloqueado o patinando, ERROR si el robot es real.
  - Pares clave-valor: mundo, version, bloqueado, patinado_m, holgura_fr/fl/rr/rl y apoyo_*.
- `/diagnostics`, DiagnosticArray a 1 Hz, como el driver.

Uso `DiagnosticStatus` y no un mensaje nuevo porque `firmware.sh` copia `starcrawler_msgs` al firmware cada vez que cambia. El mundo no debe obligar a recompilar el ESP32.

**Servicios** (std_srvs/Trigger):
- `/mundo/reiniciar`: vuelve al `inicio` del YAML.
- `/mundo/recargar`: relee el YAML, vuelve a publicar los marcadores y apoya el robot donde esté. Sirve para editar la arena sin relanzar.

**Parámetros:**
- `mundo`
- `rate_hz` = 50 (se puede bajar a 25 sin que se note)
- `wheel_radius` y `track_separation`, que el launch lee de `odometry.yaml`
- `timeout_estado_s` = 0,5
- `factor_radio` = 1,0 y `factor_giro` = 1,0, para simular errores de la odometría
- `marco_mundo` = map, `marco_odom` = odom, `marco_base` = base_link
- `publicar_imu` = true, `ruido_imu` = 0, `imu_yaw` = verdadero o relativo

**Compuertas de seguridad:**
1. **Bit 7 (hardware simulado).** No publica nada, ni siquiera los marcadores, hasta recibir un estado con el bit 7 activo. Si llega un estado sin él, se calla hasta que se reinicie el nodo y lo notifica con ERROR. Una IMU inventada junto al robot real es peligrosa en cuanto algo consuma `/imu/data`, como el nivelado de la hoja de ruta.
2. **Otro publicador en `/imu/data`** (una IMU real en el banco): deja de publicar la IMU y avisa.
3. **`/initialpose` en un marco que no sea `map`:** se ignora.

## 5. Cómo se calcula la verdad

Es el contrato con el frente del terreno. En cada ciclo, a 50 Hz:

1. **Velocidades.** `v, w = velocidades_del_robot(...)` con `wheel_radius * factor_radio`, y luego `w /= factor_giro`. Si el estado tiene más de 0,5 s, v y w valen 0. Es la misma función que usa la odometría: en llano y con los factores a 1, la verdad coincide con ella.
2. **Elevaciones.** Se toman de `crawler_angle`, que ya viene en radianes de elevación: aquí no se usa `angulos.py`. Si `encoder_ok` es falso, se usa el último valor bueno, como hace el firmware.
3. **Avance propuesto:** `ds = v*dt` a lo largo del suelo en el rumbo verdadero, y `dyaw = w*dt`.
4. **Terreno.** `terreno_core` recibe (x, y, guiñada, elevaciones, geometría) y devuelve z, cabeceo, balanceo, holguras, contactos y si el avance es admisible. Si no lo es, el avance se rechaza o se recorta y se marca `bloqueado`.
   - Condición obligatoria: en el mundo `llano`, el resultado tiene que ser idéntico a `chasis_core.pose_chasis` (test a 1e-9).
5. **Patinado:** `patinado += |ds| - |avance real|`.
6. **Publicación:** se guarda la verdad en el historial, se consulta `T_est`, se calcula `map -> odom` y se publica.

El solver tiene que tardar menos de 5 ms por ciclo en Python.

## 6. Encaje con chasis_node

- `chasis_node` sigue siendo el único dueño de las juntas `chassis_lift/pitch/roll` en todos los modos y se lanza siempre, como hoy. Representa lo que el robot cree, así que tiene que ser el mismo código que en el robot real.
- **El mundo no publica `/joint_states`.** Con dos publicadores de las mismas juntas, RSP las alterna y el modelo parpadea. Además, en simulación se dejaría de probar la cadena real.
- **Fase 1: chasis_node no se toca.** `map -> odom` absorbe el terreno completo, inclinación incluida. En `mundo.rviz` no se nota nada raro, porque todo lo que se dibuja cuelga de `map`.
- **Fase 2 (la decide Mario): un parámetro `usar_imu` en chasis_node, por defecto false.**
  - Con `/imu/data` fresca (menos de 0,2 s), el cabeceo y el balanceo salen de la IMU.
  - La altura sale de `chasis_core.altura_con_actitud()`, una función que se extrae de las últimas líneas de `pose_chasis`.
  - Si la IMU caduca, se vuelve al modelo actual.
  - Es lo que ya anuncian su docstring y la hoja de ruta de `docs/ros2.md`.
  - Con esto, `map -> odom` pasa a ser solo x, y, z y guiñada, y el robot real con IMU usará el mismo código.
  - Hoy el robot no tiene IMU (`imu_ok` está reservado), así que lo realista por ahora es no activarla.

## 7. La IMU simulada

- **Orientación:** la de base_link, porque imu_link es fija y no está girada.
  - Balanceo y cabeceo verdaderos según REP-103: cabeceo positivo es morro abajo, igual que en chasis_core.
  - Guiñada verdadera, como una IMU de 9 ejes (la MPU9250 tiene magnetómetro), o relativa si se prefiere.
- **Velocidad angular:** diferencia finita de la actitud, en los ejes de imu_link.
- **Aceleración lineal:** la gravedad en ejes del cuerpo, +9,80665 en z en reposo (REP-145).
- **Covarianzas:** diagonales y documentadas.
- **Aviso:** en el modelo cuasiestático, al caer de un canto la actitud salta de golpe. El frente de física debería limitar la velocidad de cambio a unos 60 grados/s.
- **Quién la usa:** la GUI (balanceo y cabeceo), chasis_node en la fase 2 y el futuro nivelado.

## 8. Qué pasa en cada modo

- **Robot real, sin mundo:** nada cambia. No hay `map`, RViz usa `plano.rviz` y la web dibuja con `/odom`, como hoy.
- **`sim:=true mundo:=X`:** sim_node publica lo de siempre y mundo_node lo observa.
- **ESP32 en HW_SIMULADO con `mundo:=X`:** el agente publica y el mundo observa. Aquí se añaden los sellos del micro, que absorbe el historial.
- **Robot real con `mundo:=X`:** el mundo se niega a publicar.
- **`simulate:=true`:** no está soportado, porque el simulador serie no pone el bit 7.

## 9. Launch

**En `robot.launch.py`:**
- Argumento `mundo`, vacío por defecto. Un nombre se busca como `share/starcrawler_sim/mundos/<nombre>.yaml`; una ruta a un `.yaml` se usa tal cual y así se puede editar sin `colcon build`.
- El nodo `mundo_node`, con nombre `starcrawler_mundo`, se lanza con la condición `mundo != ''`.
- La geometría la lee el launch de `odometry.yaml` y se la pasa al nodo, igual que `topes_del_mando` hace con `ds4.yaml`.
- RViz arranca con `mundo.rviz` si hay mundo y con `plano.rviz` si no.
- La GUI no necesita argumento nuevo: detecta el mundo sola.

**En `rviz.launch.py`:** un argumento `mundo:=true` para el portátil del operador.

**Mundos iniciales:**
- `llano`: vacío, para la prueba de identidad.
- `escalon`, `rampa` y `escalera`.
- `robocup`: la arena completa.

El catálogo de piezas lo define el frente del terreno.

**Arranque con el simulador:**
```
ros2 launch starcrawler_bringup robot.launch.py sim:=true mundo:=robocup rviz:=true gui_mando:=true joy_udp:=true
```

**Arranque con el ESP32:** el mismo comando cambiando `sim:=true` por `port:=/dev/ttyUSB0`, con el firmware compilado con `--simulado`.

## 10. RViz: `mundo.rviz`

**Qué lleva:**
- Marco fijo `map` y rejilla en `map`.
- El RobotModel.
- MarkerArray de `/mundo/marcadores` con Durability Transient Local. Sin eso, si RViz arranca después del mundo, no ve nada.
- MarkerArray de `/mundo/contactos`.
- Odometry de `/mundo/verdad` con Keep 1000, que dibuja el rastro verdadero.
- Display de TF apagado, con solo `map` y `odom`. Al activarlo, lo que se aleja `odom` de `map` es la deriva acumulada.
- La herramienta SetInitialPose publicando en `/initialpose`.
- Vista Orbit con Target Frame `base_footprint`.

**Qué no lleva:** `/odom`. Transformado a `map` vuelve a dibujar el rastro verdadero y engaña. Para ver cuánto miente la odometría, un opcional: `/mundo/rastro_odom`, un Path en `map` a 2 Hz igual a `T_map_odom(t0) * odom(t)`, es decir, dónde creería estar el robot si nadie lo corrigiese.

**`plano.rviz`:** no se toca.

## 11. Web (gui_node y vista3d)

- **Pose de la raíz por TF.** gui_node añade un Buffer y un Listener de tf2. Cada 0,1 s consulta `marco -> base_footprint`, donde `marco` es `map` si está disponible y `odom` si no, y guarda `ESTADO['raiz'] = {marco, xyz, q}`.
  - La página dibuja la raíz con posición y cuaternión, y la cámara y el rastro siguen también a z.
  - Si no hay `raiz`, se usa `pose`, como hoy.
  - `pose` sigue saliendo de `/odom`, porque el panel del mando lee `pose[3]` y `pose[4]`.
- **Geometría del mundo.** `/mundo/marcadores` pasa por `mundo_modelo.py`, un módulo puro como `urdf_modelo.py`, que traduce CUBE, CYLINDER y TRIANGLE_LIST a JSON. Se sirve en `GET /mundo`, que devuelve 503 si no hay mundo.
- **Estado del mundo.** `/mundo/estado` se guarda en `ESTADO['mundo']`. Cuando cambia `version`, la página vuelve a pedir `/mundo`. Se muestran BLOQUEADO, el patinado y la comparación entre la odometría y la verdad.
- **IMU.** `/imu/data` rellena el balanceo y el cabeceo de la vista 2D y `con_imu`.
  - Va fuera de `registrar()` para no contar como paquetes ni como enlace.
  - Hay que combinarlo con `cb_estado`, que escribe `con_imu=False` a 50 Hz.
- **Coste:** escuchar TF en Python, unos 200 mensajes/s, cuesta alrededor del 1 % de CPU.

## 12. Gazebo

Mantendría la decisión cerrada, pero con un matiz de hechos. Que Gazebo no modela orugas ya no es del todo cierto. gz-sim 6.2.0 y posteriores (Fortress, la versión que va con Humble) traen TrackController y TrackedVehicle, basados en el movimiento de la superficie de contacto (Pecka et al. 2017). Tienen dos pegas:
- dependen de DART;
- hubo ejemplos que no funcionaban en la versión 6.10 (issue #1662).

Las razones de fondo para no usarlo ahora son otras:
- Con el ESP32 en HW_SIMULADO, la planta ya está dentro del micro. Gazebo solo podría entrar como observador, igual que este diseño.
- `use_sim_time` no casa con el reloj de pared del ESP32.
- En WSLg el render es por software.
- Habría que ajustar cada brazo como una oruga independiente, con sus contactos en los cantos.

Merecería reabrirlo si el modelo cuasiestático se queda corto: vuelcos, caídas por la escalera o inercia. La interfaz lo permite: un mundo en Gazebo publicaría los mismos `map -> odom`, `/imu/data` y `/mundo/verdad`, y el resto del grafo no lo notaría.

## 13. Pruebas

**Pruebas puras, con pytest y sin ROS:**
- El mundo `llano` da lo mismo que `pose_chasis`.
- La corrección recompone la verdad exacta.
- Cuando la actitud estimada coincide con la real, la corrección solo tiene guiñada.
- El historial interpola bien.
- Nunca se repite un sello.
- La compuerta del bit 7 funciona.
- El paso de YAML a marcadores y de marcadores a JSON es correcto.

**Pruebas en el WSL:**
1. **Llano.** Con `llano` y 8 s de `/cmd_vel` en curva, las dos vistas tienen que marcar x = 0,63, y = 0,97 y rumbo 117 grados. `map -> odom` tiene que quedar por debajo de 5 mm y de 0,2 grados.
2. **Bloqueo.** Con el escalón y los brazos a 0: `/odom` avanza, la verdad no, salta el WARN y el patinado crece.
3. **Subida.** Levantando los brazos delanteros el robot sube: z crece lo que mide el escalón y el cabeceo de la IMU se hace negativo y vuelve a 0.
4. **Recolocar.** Con 2D Pose Estimate se coloca el robot sin reiniciar la odometría.
5. **ESP32.** Repetir 2 a 4 con el ESP32 compilado con `--simulado`.
6. **Compuerta.** Un RobotState con `error_bits` a 0 tiene que dejar el mundo en silencio.
7. **Reinicio de RViz.** Con el mundo ya en marcha, un RViz relanzado tiene que ver los marcadores.

## 14. Orden de trabajo, un commit por paso

1. `mundo_core` y el mundo `llano`: marcadores y `map -> odom` identidad, con tests.
2. El argumento `mundo` del launch y `mundo.rviz`, y la prueba 1.
3. `terreno_core`, del frente de física: bloqueo, `/mundo/estado` y contactos.
4. La IMU.
5. La GUI: raíz por TF, `/mundo`, estado e IMU.
6. El ESP32 en HW_SIMULADO.
7. Si Mario lo decide, `usar_imu` en chasis_node.

Una sección para `docs/ros2.md` la propondría, pero sin commitear, según las normas del repo.