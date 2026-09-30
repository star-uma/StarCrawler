FRENTE: LA ARENA DE PRUEBAS. Todo lo que sigue está diseñado y prototipado en el scratchpad. No he tocado ningún fichero del repo.

## 0. Resumen

La arena es un dato del PC y no depende de quién produzca el estado. Es un YAML de alto nivel (carriles, escaleras, campos de escalones…) que un módulo puro, `starcrawler_common/arena.py`, compila en una lista plana de sólidos 2,5D con `altura(x, y)` exacta y una única malla de triángulos. Esa malla la dibujan RViz (MarkerArray) y la web (three.js), y la misma `altura()` la usa quien calcule la pose sobre el terreno (el frente de física).

Funciona igual con `sim:=true` que con el ESP32 en HW_SIMULADO, porque solo consume lo que ya publican los dos (`/joint_states` y `/starcrawler/state`) y no toca ni el firmware ni `sim_core`.

El ejemplo `practica_rrl.yaml` (8 x 6 m) está pensado a escala RRL de tamaño completo: carriles de 1,2 m en zigzag y cinco niveles con los colores de las arenas NIST (amarillo, naranja, rojo). Lo he compilado y comprobado en el scratchpad (sección 8).

## 1. Qué dice el código hoy y cómo condiciona la arena

Nadie sabe hoy del terreno:
- `sim_core.py` y el ESP32 en HW_SIMULADO solo producen ángulos de brazo y velocidades de banda.
- `odometry_node` integra en 2D: la TF `odom->base_footprint` va con z = 0 y solo yaw.
- `chasis_core.pose_chasis` resuelve altura, cabeceo y balanceo suponiendo SUELO LLANO (diagonales e iteración) y los publica en `chassis_lift/pitch/roll_joint`.
- `vista3d.py` coloca la raíz en `(visto.x, visto.y, 0)` y solo gira en yaw (líneas 505-506).
- `plano.rviz` y la web pintan una rejilla de 20 x 20 m en `odom`.

Por eso la arena tiene que vivir en el PC, al lado de chasis_node.

Cotas del robot que dimensionan la arena, sacadas del URDF y de la configuración:
- Longitud con los brazos en llano: 2·(0,30+0,36+0,0415) = 1,40 m; ancho 0,56 m. Con los brazos a +90° mide 0,75 m de largo.
- Pivote a 7,6 cm del suelo. La panza del chasis (0,10 m de canto centrado en el pivote) queda a 2,6 cm: es una cota aproximada del URDF.
- Tracción máxima 0,053 m/s; brazos a 4,69°/s; recorrido 85–275 de encoder, unos ±95°.
- Radio de giro sobre sí mismo: 0,755 m con los brazos en llano (círculo de 1,51 m) y 0,469 m con los brazos a +90 (círculo de 0,94 m).
- Altura de la panza según la elevación de los cuatro brazos, en suelo llano:

| Elevación | 0° | −10° | −17,5° | −30° | −45° (preset X) | −90° |
|---|---|---|---|---|---|---|
| Panza | 2,6 cm | 5,4 cm | 10 cm | 17 cm | 24,6 cm | 35 cm |

Consecuencia: el bordillo de 10 cm ya deja colgado el robot si no baja los brazos. La física tiene que modelar el choque de la panza con el terreno.

Escala elegida:
- StarCrawler es del tamaño de Athena (0,7 x 0,5 m, 1,28 m con flippers; sube palés de 41 cm y escaleras de 45° en RoboCup 2025). Le corresponde la RRL de tamaño completo (paneles de 120 cm), no el RMRC, que está construido a escala de 30 cm.
- Según Jacoff et al., la huella del robot no debe pasar de 1/4–1/3 del palé de escalones. Con los brazos arriba (0,42 m²) sale 0,29 de un palé mediano de 1,2 m (1,44 m²). Por eso uso postes de 10 cm.

## 2. Medidas reales (C = citada, D = derivada por mí)

**Carriles**
- Bahías de 7,2 m de largo y al menos 1,1 m de ancho; hasta 2018, 1,2 m. (C, RRL 2019 v2.4)
- Los aparatos se construyen con tableros de 120 x 120 y 120 x 240 cm. (C, Qualifications RRL 2022)
- La zona de salida es de 1,2 x 1,2 m. (C, RRL 2023, Technology Challenge)

**Rampas**
- Rampas continuas y cruzadas de 15° (C, página de test methods de NIST: Yellow y Orange; RRL 2023, 2024, 2025F y 2026D).
- En 2026 los carriles de rampas cruzadas, K-rails y escalones se inclinan 0° o 15° según la dificultad, y las rampas cruzadas llevan discos deslizantes. (C, RRL 2026D)
- Elemento de rampa de 60 cm (16,1 cm de subida a 15°). (D: la RRL 2019 habla de "Elevated Ramps… 60 cm ramps" y el RMRC usa rampas de 150 x 150 mm a escala 1:4)

**K-rails**
- Vigas de 10 x 10 cm en diagonal a 45° sobre tableros de 120 x 240, de unos 161,5 cm con las puntas cortadas a 45°, en una pendiente de 15°. (C, Qualifications RRL 2022)
- "Diagonal K-Rails: 10|20 cm trip obstacles". (C, RRL 2026D)

**Bordillo**
- "Curb", barra de 10 x 10 cm en el suelo. (C, RRL 2019 MAN 6)
- En la prueba de flippers autónomos: plataforma de 10 cm. (C, RRL 2019)

**Palés y tubos (vallas)**
- Escalones de 20 cm (C, RRL 2023 a 2026D); 30 cm en la secuencia "Dwelling Access" (C, RRL 2026D).
- "A 20 cm tall rolling pipe obstacle" (C, RRL 2019 MOB 1).
- En el RMRC, tubos de 50 mm que forman el propio canto, giran libres, con dos subidas de 50 mm y una bajada de 100 mm (C, OARKit hurdles). A escala 4:1 salen +20, +20 y −40 cm con tubo de 20 cm (D).

**Escaleras**
- 35°, 40° y 45°, con 0, 2 o 4 barreras (C, RRL 2023 a 2026D).
- Contrahuella de 20 cm ("Stair Debris have 20 cm risers"). (C, RRL 2026D)
- NIST las prueba de 30° a 45°, en madera o metal y mojadas. (C, página de test methods de NIST)
- Huellas de 28,6, 23,8 y 20,0 cm (D: 0,20/tan θ).
- ASTM E2804 prevé holguras laterales de 2,4, 1,2 y 0,6 m. (C, resumen público de la norma)

**Campos de escalones (stepfields)**
- Palé pequeño: 60 cm con postes de 5 cm. Mediano: 120 cm, rejilla de 10 x 10 postes de 10 cm, alturas de 10/20/30/40 cm. Grande: 240 cm con escalones de 20 cm (20–80 cm). (C, Jacoff et al. 2008)
- El marco de contención suma dos unidades en cada dirección. (C, Jacoff et al. 2008)
- Regla 1: nunca más de 2 unidades de salto entre vecinos. (C, Jacoff et al. 2008)
- Los planos llevan 4 postes de 3 unidades; las colinas, una cresta de 4 unidades con filas de 2–3 unidades a los lados. (C, Jacoff et al. 2008)
- "Half-cubic" (naranja) es la misma configuración con media altura. (C, Jacoff et al. 2008)
- "Half-Cubic Stepfields: 15 cm step heights". (C, RRL 2026D)
- El Stepfield Dash (5 palés en línea) se estrenó en la German Open de 2005. (C, Jacoff et al. 2008)

**Otras**
- Traverse: plano inclinado de 30° en 2019 y de 15° desde 2023. (C)
- "Avoid": camino de palés con cambios de nivel de 10 cm y 5 puertas de 90 cm. (C, RRL 2026D)
- Center in Doorways: ancho del robot + 10 cm. (C, RRL 2026D)
- NIST: huecos y pipe step de 10 a 100 cm, plano inclinado de 0° a 90°. (C, página de test methods de NIST)

**German Open**
- No he encontrado planos propios. En 2025 fue en Núremberg (13–16 de marzo, NürnbergMesse) con la RRL; las cotas que valen son las comunes de las reglas RRL y de NIST.
- Las normas ASTM (E2826, E2827, E2828 y E2804) son de pago: sus plantillas exactas no están aquí.

## 3. Modelo: mundo 2,5D con tres sólidos básicos

Todo compila a tres tipos:
- `bloque`: rectángulo orientado (centro, rumbo, largo, ancho) con techo bilineal definido por la altura de sus 4 esquinas, macizo hasta el suelo. Una caja tiene las 4 alturas iguales; una rampa, 2 y 2; una viga apoyada en una pendiente, las de la pendiente más su canto.
- `cilindro`: tubo tumbado.
- `hueco`: fuerza z = −fondo.

La altura en cada punto es `altura(x, y) = min(huecos, max(0, techos de bloques y cilindros))`.

Qué deja fuera el 2,5D:
- Lo que tiene voladizo: el confined space de NIST, el "crouch under landing" de 2026 y los "Negotiate" colgados de un travesaño.
- La mitad inferior de los tubos, que en el mapa de alturas se ve como una cara vertical. Por eso el cilindro se conserva como primitiva propia: la física puede usar el círculo entero.

API de `arena.py` (puro, como `angulos.py` y `orugas.py`):
- `cargar(texto_yaml) -> Arena`
- `Arena.altura(x, y, con_paredes=True)`
- `Arena.perfil(p0, p1, n)`
- `Arena.solidos` (cada uno con color, tramo, pared y agarre)
- `Arena.paredes()` (para choques)
- `Arena.malla() -> (vértices, triángulos, color por vértice, aristas)`
- `Arena.inicio`, `Arena.tramos`, `Arena.zonas`

Los errores son `ErrorArena` con mensajes del tipo "carril_3, elemento 2 (viga): …".

Índice espacial: celdas de 0,25 m. Medido con el ejemplo (Python 3.14, Windows): 405 sólidos, 1,2 µs por consulta de media y 7,6 µs dentro del campo de escalones (hasta 16 candidatos por celda). Con eso no hace falta rasterizar ni numpy. Falta medirlo con el Python 3.10 del WSL.

## 4. Primitivas del fichero

**Comunes a todas**
- `pos: [s, t]`: centro del borde de ENTRADA en el marco del carril (s a lo largo en el sentido de avance, t lateral, + = izquierda del robot); el elemento se extiende hacia +s.
- `pos: [sigue, t]`: empieza donde acabó el anterior.
- Además: `rumbo` (°, relativo al carril), `ancho` (por defecto, el del carril), `z` (cota de la base), `color`, `agarre` (0–1, pista para la física; los tubos llevan 0,3) y `encima: true` (se apoya en lo que haya debajo; por ejemplo K-rails sobre 15°; probado: la viga queda a rampa + 0,10).
- Los carriles llevan `nombre`, `titulo`, `nivel`, `color`, `origen [x, y]`, `rumbo`, `largo`, `ancho` y `elementos`.

**Primitivas**
- `caja`: `largo`, `alto`; opcionales `tubo: d` / `tubo_salida: d` (pone un tubo de diámetro d en el canto de entrada o de salida, enrasado arriba). Sirve para escalones, palés, plataformas y "pallets & pipes".
- `rampa`: exactamente dos de `largo` / `alto` / `pendiente`; con `alto` negativo baja desde `z`. Rampas de 15°, traverse, plano inclinado; con `rumbo: 90`, peralte lateral.
- `escalera`: `peldanos`, `contrahuella`, `huella` o `pendiente`, `sentido: sube|baja`, `rellano`, `bajada: {huella|pendiente}`. Una sola entrada describe la pirámide sube–rellano–baja.
- `campo_escalones`: `celda`, `unidad`, `base`, y `alturas` (matriz: fila = a lo largo desde la entrada, columna = de izquierda a derecha del robot) o `patron` (`colina_diagonal`, `colina_perpendicular`, `plano_cruz`, `pico_central`) con `filas` y `columnas`. La regla de 2 unidades se comprueba al cargar. Los patrones son simétricos e inspirados en E2828, no son la norma.
- `campo_rampas`: `celda`, `pendiente`, y `orientaciones` (A/T/I/D: hacia dónde sube cada celda) o `patron: cruzadas|continuas`. Cubre E2827 y E2826.
- `viga`: `de: [s, t]`, `a: [s, t]`, `seccion`, `alto`, `trozo`; se recorta al carril. Sirve para bordillos, K-rails y barreras de escalera.
- `tubo`: `diametro`. Tubo suelto en el suelo (hurdle).
- `zanja`: `largo`, `fondo`. El "Gap" de NIST.

**Nivel superior**
- `paredes`: polilíneas con `alto` y `grosor`; para la física son infranqueables.
- `zonas`: solo pintura en el suelo.
- `recorrido`: orden de los tramos, para un futuro cronómetro.
- También `limites`, `inicio {x, y, rumbo}`, `marco: arena` y `colores`.

## 5. YAML de ejemplo

Fichero: `C:\Users\mario\AppData\Local\Temp\claude\C--Users-mario-Documents-TFM\084ea808-c449-41a1-a142-794f1198f2ca\scratchpad\arena\practica_rrl.yaml` (181 líneas; en el repo iría a `starcrawler_description/arenas/practica_rrl.yaml`).

**Trazado**
- Paredes: exterior de 8 x 6 m y tabiques en y = 1,2 (x de 0 a 6,4), 2,4 (de 1,6 a 8), 3,6 (de 0 a 6,4) y 4,8 (de 1,6 a 8); 2 cm de grosor y 0,8 m de alto.
- Zonas: salida [0–1,6] x [0–1,2]; bahía A [6,4–8] x [0–2,4]; bahía B [0–1,6] x [1,2–3,6]; bahía C [6,4–8] x [2,4–4,8]; meta [0–1,6] x [3,6–4,8].
- `inicio`: (0,80, 0,60), rumbo 0.
- Recorrido: salida → carril 1 → A → carril 2 → B → carril 3 → C → carril 4 → meta. Son unos 26 m, 8,2 min solo de avance a 0,053 m/s; con las maniobras de brazo, bastante más.

**Carriles** (1,2 m de ancho; los cuatro primeros miden 4,8 m)

- **Carril 1** (amarillo; origen (1,6, 0,6), rumbo 0):
  - viga de 10 x 10 cm atravesada en s = 0,45 (el curb);
  - rampa de 15° hasta 0,16 m en s = 1,1, meseta de 0,6 m con `sigue` y rampa de bajada con `sigue`;
  - rampas cruzadas 2 x 2 de celdas de 0,6 m a 15° en s de 3,3 a 4,5.
- **Carril 2** (naranja; origen (6,4, 1,8), rumbo 180):
  - caja de 0,10 en s de 0,4 a 1,4;
  - caja de 0,20 en s de 2,0 a 3,0;
  - valla: caja de 0,20 en s de 3,4 a 4,0 y de 0,40 en s de 4,0 a 4,6, las dos con `tubo: 0.20`, y bajada de 40 cm.
- **Carril 3** (naranja; origen (1,6, 3,0), rumbo 0):
  - K-rails a 45°: viga de 10 cm de (0,3, 0,6) a (1,5, −0,6) y de 20 cm de (0,9, 0,6) a (2,1, −0,6);
  - escalera en s = 2,5: 3 peldaños de 20 cm a 35° (huella 0,286 m), rellano de 0,8 m a 0,60 m de altura y bajada a 45° (huella 0,200 m), en rojo.
- **Carril 4** (rojo; origen (6,4, 4,2), rumbo 180):
  - campo medio cúbico de 12 x 12 celdas de 0,10 m con unidad de 0,05, `plano_cruz`, en s de 0,3 a 1,5;
  - campo cúbico de 12 x 12 celdas de 0,10 con unidad de 0,10 y matriz explícita en colina diagonal (cresta de 4 unidades = 40 cm; nunca más de 1 unidad entre vecinas) en s de 1,9 a 3,1;
  - zanja de 0,30 m con fondo 0,5 en s = 3,6.
- **Carril 5** (zona libre, azul; origen (1,6, 5,4), largo 6,4 m): cajas de 0,15, 0,30 y 0,40 m, de 0,6 m de largo, en s = 0,4, 2,2 y 4,0. La de 0,40 es el límite, como el palé de 41 cm de Athena.

El fichero lleva comentada la fuente de cada elemento.

## 6. Cómo se engancha sin cambiar lo que ya funciona

**Ficheros nuevos**
- `starcrawler_common/arena.py`, con `test_arena.py`: alturas conocidas, peldaños, recorte de vigas, `encima`, errores y carga de todas las `arenas/*.yaml` si se encuentran. Añadir `<exec_depend>python3-yaml</exec_depend>`.
- `starcrawler_description/arenas/{practica_rrl,llano}.yaml` y añadir `arenas` al `install(DIRECTORY …)` del CMakeLists.
- `starcrawler_sim/arena_node.py`.
- `arena_check`, un ejecutable que valida un YAML e imprime el resumen: contención, carriles, bahías llanas, holgura de giro y saltos por cada oruga. Es el `validar.py` del prototipo sin los dibujos.

**arena_node** (parámetros `arena`, que es un nombre o una ruta, y `recargar`, por defecto true)
- Lee y valida el YAML.
- Publica `/arena/descripcion` (`std_msgs/String`, TRANSIENT_LOCAL) con el texto YAML ya validado, igual que `/robot_description`: cada consumidor compila con el mismo `arena.py` y todos ven exactamente lo mismo.
- Publica `/arena/marcadores` (`MarkerArray`, TRANSIENT_LOCAL, frame `arena`):
  - `solidos` y `paredes` como TRIANGLE_LIST con color por vértice, las paredes con alfa 0,35;
  - `aristas` como LINE_LIST, porque el sombreado de TRIANGLE_LIST en RViz no garantiza que se distingan los escalones;
  - `suelo` y `zonas`, planos a +1 mm;
  - `rotulos`, TEXT_VIEW_FACING con el título de cada carril.
- Emite la TF estática `arena -> odom` a partir de `inicio`.
- Con `recargar`, mira el mtime cada segundo y republica si el fichero cambia y sigue siendo válido; si no es válido, deja el anterior y da el error por el log. Así Mario diseña la pista en VS Code con RViz abierto.
- Comprueba el bit 7 de `error_bits`: con el robot REAL, la arena es ficticia y lo dice.

**Launch**
- `arena:=` vacío por defecto: nada cambia respecto a hoy.
- `arena:=practica_rrl` añade `arena_node`, con cualquier fuente de estado (sim, micro-ROS o driver).

**RViz**
- Añadir a `plano.rviz` un display MarkerArray (`/arena/marcadores`, Transient Local).
- Con arena, el marco fijo recomendado es `arena`. La rejilla se puede referir a `arena` con offset (4, 3).

**Web**
- `gui_node` se suscribe a `/arena/descripcion` y sirve `/arena` con la malla en JSON (unos 222 KB para el ejemplo: 4058 triángulos con paredes).
- Mete un `arena` (hash) en `/events` para que la página vuelva a pedirla al recargar.
- `vista3d` crea un único BufferGeometry, con la malla en coordenadas de la arena dentro de un grupo colocado en inversa(inicio) respecto a `odom`. Es el mismo principio que `urdf_modelo.py`: Python traduce y JS dibuja, así que RViz y la web pintan lo mismo.

**Lo que la arena exige a los otros frentes** (la arena no lo resuelve)
1. Una pose real sobre el terreno. La odometría seguirá integrando las bandas y atravesaría un escalón de 40 cm o una pared, porque ni el ESP32 ni `sim_core` saben del terreno. Propuesta de interfaz: la verdad se publica como `arena -> odom` dinámica (patrón `map -> odom` de REP-105) y `/odom` se queda tal cual; así además se ve la deriva.
2. z, cabeceo y balanceo del cuerpo en las vistas. La web hoy fija z = 0; el límite de `chassis_lift_joint` en el URDF es de 0,4015 m y el rellano está a 0,60.
3. El choque de la panza, necesario incluso para el bordillo.
4. Las paredes como choque, no como terreno trepable: `altura(…, con_paredes=False)` y `paredes()`.

## 7. Gazebo

Propongo mantenerlo descartado para este frente, pero la premisa hay que matizarla: gz-sim Fortress, la pareja de Humble, sí tiene TrackedVehicle y TrackController ("contact surface motion", el método de Pecka et al.), aunque con problemas conocidos (gz-sim #1662: los mundos de ejemplo no funcionaban).

Razones para no reabrirlo:
- Con HW_SIMULADO la planta vive en el ESP32; Gazebo sería una segunda planta y rompería la equivalencia entre sim y ESP32, que es requisito.
- A 5 cm/s y 4,7°/s el régimen es cuasiestático: la dinámica solo aporta vuelcos y patinazos.
- El formato es agnóstico del simulador. Si algún día se reabre, pasar los bloques a SDF (box o una malla) es un script corto.

## 8. Verificado frente a solo escrito

Verificado en el scratchpad con Python 3.14 (`arena_proto.py`, `validar.py`, `giro.py`):
- El YAML compila: 405 sólidos (402 bloques, 2 cilindros, 1 hueco) y ninguno fuera de la arena.
- Cada carril usa s de 0,3–0,4 a 4,5–4,76 de sus 4,8 m (el 5, hasta 4,6 de 6,4) sin salirse de |t| = 0,6.
- Las cinco zonas quedan llanas.
- Los saltos por las orugas (t = ±0,24) son los esperados: bordillo +0,10; escalones +0,10 y +0,20; tubos; bajada de −0,40; peldaños de 0,20; medio cúbico ±0,05; zanja −0,5.
- Giro: en el centro de las bahías hay 0,79 m de holgura, así que el robot gira con los brazos en llano (le bastan 0,755). En la salida, la meta y los carriles hay 0,59 m: solo gira con los brazos a +90.
- Tres YAML erróneos dan un mensaje claro: rampa con tres cotas, salto de 3 unidades y tipo desconocido.
- `encima` sobre una rampa de 15° funciona.
- La malla sale con 4058 triángulos.
- Dibujos para revisar a ojo: `planta.png`, `perfiles.png` y `vista3d.png`, en la misma carpeta que el YAML.

NO verificado: nada en ROS (ni el nodo, ni RViz, ni la web), el rendimiento en el Python 3.10 del WSL, y las primitivas giradas (`rumbo` distinto de 0 en campos de escalones o de rampas no está implementado en el prototipo).