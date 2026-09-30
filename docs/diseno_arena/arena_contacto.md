CONTACTO ORUGAS-TERRENO: chasis_core generalizado a terreno 2,5D, cuasiestático y sin física

No he tocado nada del repo. Todas las cifras de abajo salen de un prototipo 2D (vista lateral, solo cabeceo) que escribí en el scratchpad con la geometría del URDF. En ese prototipo aparecieron dos fallos de concepto que el diseño corrige (4.3 y 4.4). Lo 3D (balanceo, guiñada, líneas laterales) está diseñado pero sin prototipar.

0. RESUMEN
Propongo un módulo puro nuevo, contacto_core.py, que en cada tick hace tres cosas:
(1) Calcula dónde toca cada pieza (4 orugas y chasis) sobre perfiles del terreno. La oruga es la envolvente exacta de sus dos poleas, así que un canto puede apoyar en cualquier punto de la banda.
(2) Coloca el chasis con un "plano de apoyo con brazos de palanca", que generaliza la regla de las diagonales de chasis_core: el CdG queda dentro del polígono de apoyo. Si se sale, el robot vuelca sobre la arista hasta el primer contacto nuevo, a 90°/s como mucho.
(3) Avanza la posición plana con la cinemática de las orugas. Un contacto "de pared" (normal a más de 75° de la vertical) empuja la base hacia atrás a lo largo del rumbo, y eso es el bloqueo.
El módulo solo observa brazos y tracción y no realimenta nada, así que con sim:=true y con el ESP32 en HW_SIMULADO se comporta igual.

1. ENCAJE (lo mínimo; la integración es de otro frente)
- Ficheros: starcrawler_sim/contacto_core.py (este frente), starcrawler_sim/terreno_core.py (primitivas; del frente de la arena; solo le pido perfil()) y un nodo mundo_node que sustituye a chasis_node cuando hay terreno.
  - Los dos no pueden correr a la vez, porque ambos publicarían chassis_lift/pitch/roll_joint.
  - Reutiliza chasis_core.Geometria y geometria_desde_urdf. Añade lo que falta del URDF: la caja del chasis (0,60 x 0,40 x 0,10), el ancho de oruga (0,08, del length del cilindro) y las masas (base_mass 20 kg; arm_mass 3 kg con su CdG a L/2 del pivote).
- Entradas: las elevaciones {FR, FL, RR, RL} de /joint_states y la propuesta plana (v, w). Recomiendo sacar (v, w) de los incrementos de /odom, para que la cinemática (wheel_radius 0,0764, track_separation 0,524) viva en un solo sitio.
- Salidas: la pose 6D de base_link en el mundo (x, y, guiñada, z, cabeceo, balanceo; ZYX, que es el orden de las juntas virtuales, con el convenio REP-103 de chasis_core: cabeceo + = morro abajo), los contactos y las banderas.
- Por qué solo observador: el ESP32 no sabe del terreno. Si con sim:=true el mundo frenase brazos o tracción, los dos caminos divergirían. Consecuencia buscada: contra una pared las orugas patinan, igual que en el robot real, donde los RMD no ven la pared. /odom sigue avanzando, y la diferencia odom-mundo es el deslizamiento.
- Aviso para integración:
  - la web coloca la raíz en (x, y, 0) desde /odom (vista3d.py, bucle), así que tiene que pasar a usar la pose del mundo;
  - chassis_lift_joint tiene upper = 0,4015 en el xacro y en escaleras llegaría a ~1 m. robot_state_publisher y urdf_modelo no la acotan, creo; conviene comprobarlo, o ensanchar el límite, o llevar z en la TF.

2. GEOMETRÍA (números del URDF actual)
- Oruga: envolvente convexa de dos círculos en el plano x-z del brazo: la polea activa R1 = 0,0764 en el pivote y la pasiva R2 = 0,0415 a L = 0,36. El tramo inferior va inclinado β = asin((R1−R2)/L) = 5,563° respecto al brazo; es el belt_angle del xacro.
  - q = 0: solo apoya la polea activa.
  - q = −β: apoya el tramo entero.
  - q < −β: apoya la punta y el chasis sube (lo que ya hace chasis_core).
- Dos operaciones exactas y baratas sobre la envolvente:
  - Punto de soporte en una dirección n: c_i + r_i·n, del círculo que maximiza c_i·n + r_i. De ahí sale el contacto con un tramo de terreno de cualquier pendiente.
  - Borde inferior b(s): el mínimo de los dos arcos inferiores y las dos tangentes exteriores. Vale también con el brazo vertical, donde "la cadena de abajo" es solo la polea activa.
- Chasis: rectángulo de 0,60 x 0,10 en x-z. La panza queda a −0,05 de base_link y a 0,0264 del suelo en reposo. Pivotes en (±0,30, ±0,24).
- Paso a 3D por líneas: cada pieza se evalúa en planos verticales paralelos al rumbo.
  - 2 líneas por oruga, en y = ±0,24 ± 0,035; 5 en el chasis, en y = −0,19, −0,095, 0, 0,095, 0,19. En total 13.
  - En cada línea el terreno es un perfil 2D: una polilínea (s, z) con s creciente, donde un salto vertical son dos vértices con la misma s.
  - El balanceo entra por la altura relativa de las líneas. Se desprecia la inclinación de los círculos por el balanceo: el error lateral es z_b·sen(balanceo), unos 4 cm con la punta bajada a −45° y 10° de balanceo.
  - Un obstáculo más estrecho que la separación entre líneas (~9,5 cm en el chasis) se puede colar entre ellas.

3. CONTACTO EN UNA LÍNEA
- Rasgos del terreno: tramos (horizontales o inclinados), caras verticales y esquinas convexas (los cantos).
- Para cada rasgo cercano se calculan tres cosas:
  - la holgura vertical: cuánto hay que subir la pieza para no atravesarlo; en 2,5D, un valor positivo significa que penetra;
  - el punto de contacto;
  - la normal terreno→robot (en una esquina, la normal de la pieza en ese punto).
- Ángulo de ataque φ: ángulo entre esa normal y la vertical. Clasificación:
  - APOYO si φ ≤ 75° en banda o φ ≤ 60° en chasis. Se resuelve en vertical: el chasis sube o cabecea.
  - PARED si pasa de ese umbral. Se resuelve en horizontal: se empuja la base a lo largo del rumbo.
  - TRACCIÓN: solo la dan las bandas. En esquinas, siempre (la banda engancha el canto); en tramos lisos, solo si la pendiente es ≤ 40°. El chasis nunca da tracción.
- Ejemplos: el canto sobre el tramo inferior tiene φ = q + β + cabeceo hacia arriba. El frente de la polea pasiva contra una cara tiene φ = 90°, que es pared.

4. REPOSO: ALTURA, CABECEO Y BALANCEO
4.1 Plano de apoyo con brazos de palanca
- Candidatos: los contactos de apoyo, con su altura de base requerida H_k (en la orientación actual) y su posición horizontal p_k.
- Se busca el plano P(p) = a + b·(p − g), con g el CdG en planta, que cumple P(p_k) ≥ H_k en todos los contactos y deja P(g) mínimo.
  - Es la envolvente convexa superior de los puntos evaluada en el CdG, un programa lineal cuyos pesos duales son las cargas de cada contacto.
  - Sus tres contactos forman el triángulo de apoyo que contiene al CdG. Se enumeran ternas entre los 10 candidatos más altos (a menos de 6 cm del máximo), y antes se prueba el triángulo del tick anterior.
- La pendiente b da la corrección: cabeceo −= b·rumbo y balanceo += b·lateral. Se aplica con relajación 0,5 (como chasis_core), a lo sumo 5° por iteración y 8 iteraciones por tick, arrancando en caliente desde el tick anterior.
- Al final siempre z = max H_k: nunca penetra en vertical, aunque no haya convergido.
- Empate (el CdG sobre una arista, con dos caras óptimas): se promedian las dos caras. Con los 4 pivotes y el CdG en el centro, esto es exactamente la regla de las diagonales de chasis_core, incluidas las "otras dos igual de levantadas".
4.2 Paridad en llano (medida)
Comparé con chasis_core en 9 poses: (0), (45), (−45), (−90), (−30,−30,0,0), (0,0,−30,−30), (45,45,−45,−45), (−30,−30,10,10) y (−20,−20,−60,−60).
- El cabeceo coincide al 0,01°: −8,056 / +8,056 / +14,345 / −9,016 / +9,575.
- La altura coincide a 0,4 mm. Esos 0,4 mm son el polígono de 24 lados del prototipo; con círculos exactos es la misma geometría.
- Única diferencia: una sola oruga bajada (FR a −30°). Ahora el contacto está en la punta, en x = 0,61 y no en el pivote, así que el robot apoya en FR, RR y RL con FL al aire. En chasis_core quedaba sobre la diagonal FR-RL con FL y RR igual de levantadas. Lo nuevo es más correcto, pero cambia ese test.
4.3 Primer fallo del prototipo: no minimizar el CdG con la planta fija
Minimizar la altura del CdG dejando (x, y) fijos y "z = elevación necesaria" deja que el contacto resbale sobre el canto sin rozamiento. El robot quedó "en equilibrio" sobre la panza con el CdG 1,6 cm por detrás del canto y sin tracción. Por eso los brazos de palanca van fijos en cada iteración.
4.4 Vuelco (el CdG fuera del apoyo, o a menos de 1 mm del borde sin nada más allá)
- El sólido gira sobre la arista que ha cruzado el CdG (o sobre el contacto único, con eje horizontal). El pivote queda fijo en el mundo, así que se mueven x, y, z y algo la guiñada, como en la realidad.
- Gira solo hasta el primer contacto nuevo (bisección en el ángulo, 10 mitades), y como mucho caida_ang·dt, 90°/s, es decir 1,8° por tick.
- Segundo fallo del prototipo: con paso fijo de 0,5° se pasaba, la punta trasera se clavaba en el canto y aparecía un ciclo límite (avanza 1 mm, vuelve 0,64 mm) al final de una bajada.
- Girando sobre base_link en vez de sobre el pivote, el equilibrio inestable se vuelve neutro y el robot se queda en el canto sin tracción.
- Caída sin ningún contacto: z baja a 1 m/s como mucho.
- VOLCADO: si |cabeceo| o |balanceo| pasa de 75°, el robot se congela con la bandera puesta.
- colocar(): hace lo mismo sin límite por tick y girando sobre base_link. No mueve (x, y, guiñada) al posar o reiniciar.

5. AVANCE Y BLOQUEO (por tick; validado en 2D salvo la guardia lateral)
(a) Subpasos: n = ceil(max(|v|dt, |w|dt·0,75, max|Δq|·0,40) / 0,002). Nada se mueve más de 2 mm por subpaso. Normalmente sale 1: 40 dps son 1,07 mm/tick y los brazos, a 4,69°/s, mueven la punta 0,6 mm/tick. Con w = 0,2 rad/s salen 2.
(b) Tracción: hace falta algún contacto de banda en el apoyo con tracción y holgura ≤ 2 mm. Sin tracción no hay avance ni giro: la panza sobre un canto o una barra, con las orugas al aire.
(c) Propuesta plana: odometry_core.integrar(x, y, yaw, v·cos(cabeceo), w, dt). En llano coincide exactamente con /odom; en rampa avanza v·cos θ en planta.
(d) Guardia anti-teleporte (3D): si la altura requerida de una línea sube más de d·tan 75° + 2 mm (d = lo que se ha movido esa línea en planta, incluido lo que la mueve el brazo), se deshace por bisección (6 mitades) la parte plana del subpaso y se marca bloqueado.
  - Pilla lo que las normales en el plano de la línea no ven: entrar de lado en una pared al girar o al acercarse en oblicuo.
  - Bajar un brazo sobre algo siempre está permitido, porque la punta se mueve d = L·Δq.
(e) Reposo (sección 4), que empieza y acaba con EL EMPUJE:
  - Los contactos de pared que penetran se sacan moviendo la base a lo largo del rumbo, a lo sumo 5 mm por subpaso. Sin esa cota, en el prototipo, colocar un robot mal apoyado lo teletransportaba 0,49 m.
  - Si hay empujes en sentidos opuestos, se deshace el subpaso (atrapado).
  - Al conducir contra la pared, el empuje devuelve lo avanzado y la base se queda a 0,01 mm de la cara, sin necesidad de bisección.
  - Si un brazo se mete en una pared, el robot retrocede, porque el brazo no se puede frenar (lo manda el ESP32).
(f) bloqueado = el empuje o la guardia se han comido más de la mitad del avance propuesto. Se devuelve qué pieza.

Ley resultante del primer contacto, con el chasis en llano y φ_b = 75° (verificada con el prototipo al 0,1 mm):
- si q + β ≤ φ_b: h_max(q) = R1 + L·sen q − R2·cos φ_b;
- si no: R1·(1 − cos φ_b);
- con las puntas en el suelo: R2·(1 − cos φ_b) = 0,031.

| q | h_max |
|---|---|
| 0° | 0,0657 |
| 10° | 0,1282 |
| 20° | 0,1888 |
| 30° | 0,2457 |
| 45° | 0,3202 |
| 60° | 0,3774 |
| 69° | 0,4017 (máximo, en q = 69,4°) |
| ≥ 70° | 0,0566 (la polea activa encara el escalón) |

Al bloquearse, la base se para a x_cara − (0,30 + 0,36·cos q + 0,0415): 0,7015 con q = 0, 0,6798 con 20° y 0,6533 con 30°.

6. RESULTADOS DEL PROTOTIPO 2D
Condiciones: φ banda 75°, φ chasis 60°, masas del URDF, 40 dps, brazos a 4,69°/s y brazos traseros a 0 salvo que se diga otra cosa.

Escalón con los brazos delanteros fijos:

| h | Sube con q | Falla |
|---|---|---|
| 0,05 | cualquiera | — |
| 0,15 | 20 a 60° | 0 y 10 (primer contacto), 70 (brazo casi vertical) |
| 0,20 | 30, 40, 50° | 20 (primer contacto), 60 (atasco a media subida con cabeceo 9,6°: el tramo supera 75°) |
| 0,25 | 40, 50° | 30 (primer contacto), 60 (atasco) |
| 0,30 | solo 50° | — |
| 0,35 | ningún ángulo fijo | — |

- En 0,35 m el guion "brazos a 60°, y al pasar de 8° de cabeceo bajarlos a 20°" sí sube, con cabeceo máximo de 34,5°. Es la coordinación de la competición.
- Cuando sube, acaba con z = h + 0,0764 y cabeceo 0.

Bajada de escalón:
- 0,10 / 0,20 / 0,30 m con brazos a 0°: llega al suelo con cabeceo mínimo de −9,5 / −18,3 / −27,6°. El vuelco toca el tope de 90°/s.
- Con brazos a −20° y −40° también baja, y termina apoyado sobre las puntas.

Rampas:
- 15°, 30° y 45° suben con cabeceo igual a la pendiente.
- Al prototipo le falta la regla de tracción: con ella la de 45° lisa no sube.

Escalera de 4 peldaños (brazos delanteros a 45° para engancharla y cambio al pasar de 15° de cabeceo):

| Escalera | Sube con | Se atasca con |
|---|---|---|
| 0,17/0,29 (30,4°) | (−6,−6) y (−15,−15) | (−6, 0) |
| 0,20/0,25 (38,7°) | (−6,−6) (cabeceo máx 38,6°) y (−15,−15) | (−6, 0); brazos fijos a 30° (la punta contra la tabica, 90°); 45→0 (la polea activa trasera contra la nariz a 77°) |
| 0,20/0,20 (45°) | solo (−15,−15) | (−6,−6) (se queda en la última nariz) |

Es la técnica de competición: alinear los cuatro brazos con la pendiente.

Coste: 1,7 a 2,7 ms/tick, con polígonos y sin optimizar.

7. API (pura, sin ROS)
PIEZAS = ('FR','FL','RR','RL','chasis')

@dataclass(frozen=True) class GeometriaRobot:
    orugas: chasis_core.Geometria
    chasis: Tuple[float,float,float]
    ancho_oruga: float
    masa_chasis: float
    masa_brazo: float

def geometria_robot_desde_urdf(xml: str) -> GeometriaRobot

@dataclass(frozen=True) class Parametros:
    ataque_banda = rad(75); ataque_casco = rad(60); traccion_lisa = rad(40)
    caida_ang = rad(90)  # rad/s
    caida_lin = 1.0      # m/s
    vuelco = rad(75)
    subpaso = 0.002; empuje_max = 0.005
    tol_apoyo = 0.0005; tol_traccion = 0.002; eps_equilibrio = 0.001
    lineas_oruga = (-0.035, 0.035); lineas_chasis = (-0.19, -0.095, 0.0, 0.095, 0.19)
    iteraciones = 8

@dataclass(frozen=True) class Pose3D:
    x; y; yaw          # base_footprint en el mundo
    z; cabeceo; balanceo  # base_link

@dataclass(frozen=True) class Contacto:
    pieza; x; y; z; normal: Tuple[3]; ataque; tipo: 'apoyo'|'pared'; traccion: bool; holgura

@dataclass(frozen=True) class Estado:
    pose: Pose3D; elevaciones: Tuple[4]; contactos: Tuple[Contacto,...]
    apoya: Tuple[4 bool]; bloqueado: Optional[str]
    sin_traccion; cayendo; volcado; soporte: Tuple[int,...]  # soporte es el arranque en caliente

def colocar(x, y, yaw, elevaciones, terreno, geo, par=Parametros()) -> Estado
def paso(estado, v, w, elevaciones, dt, terreno, geo, par=Parametros()) -> Estado
    # v en m/s y w en rad/s, como odometry_core.velocidades_del_robot
def altura_trepable(q, geo, par) -> float        # la ley cerrada
def contactos_linea(forma, perfil, par) -> list  # expuesta para los tests

Lo que pido a terreno_core: Terreno.perfil(x0, y0, ux, uy, s0, s1) -> [(s, z)], en menos de ~20 µs por línea con ≤ 10 primitivas cercanas (rejilla espacial).

8. TESTS (pytest sin ROS)
Usan una geometría de test fija, como el GEO de test_chasis_core, para no romperse cuando el URDF reciba las cotas del CAD.

A) test_contacto_geometria.py
- β = asin(0,0349/0,36) = 5,563°, igual al belt_angle del xacro.
- Soporte hacia abajo: con q = 0 sale la polea activa (0,30, −0,0764); con −10°, la punta; con −β, empate a 1e-9.
- Borde inferior con q = 0: b(0,30) = −0,0764 y b(0,66) = −0,0415. Con q = 90°, solo el arco de R1.
- Esquina sobre el tramo con q = 30°: ataque de 35,56°.
- El xacro real (importorskip): chasis 0,60 / 0,40 / 0,10, ancho 0,08 y masas 20 / 3.

B) test_contacto_reposo.py
- Llano: z = 0,0764, sin inclinación y apoyan las 4. Con −45° z = 0,29606; con −90° z = 0,4015.
- Paridad con chasis_core en las 9 poses de 4.2: ±0,05 mm y ±0,01°.
- Una sola bajada (FR a −30°): apoyan FR, RR y RL (cambio documentado).
- Rampa de 15° con brazos a −β: cabeceo −15,00 ± 0,05°.
- Caja de 0,05 bajo las dos orugas izquierdas: balanceo +5,95 ± 0,3°.
- Barra de 0,10 x 0,12 bajo la panza con brazos a +30°: sin_traccion y z = 0,17. Con brazos a −30°: de pie sobre las 4 puntas, z = 0,2215, con tracción.
- Propiedad, 600 casos al azar (cajas y rampas ≤ 0,3 m, q entre −90° y 90°): ninguna línea penetra más de 1e-6 y, si no vuelca, el CdG cae dentro del apoyo (±1 mm).
- colocar no mueve (x, y, guiñada).

C) test_contacto_avance.py
- Llano igual a odometría: 1000 pasos con v = 0,053 y w = 0,2 dan (x, y, guiñada) iguales a odometry_core.integrar a 1e-9.
- Primer contacto, parametrizado con q en {0, 10, 20, 30, 45, 60, 69}:
  - h_max − 5 mm: sube (el morro gana ≥ 1 cm en 2 s);
  - h_max + 5 mm: se para a la distancia de la sección 5 (±1 mm), con bloqueado = 'FR'/'FL' y z sin cambiar.
- q = 72°: sube 0,050 y no 0,062.
- Pared de 1 m: con q entre −20° y 90° no pasa. Tras 10 s empujando, x quieta ±1 mm mientras la odometría avanza 0,53 m.
- Tablas de la sección 6 como regresión:
  - 0,20 m con 30/40/50° sube; con 20° se para en x = 0,320 con la cara en 1,0; con 60° se atasca con cabeceo 9,6 ± 1°;
  - 0,35 m: con 60° fijo no sube; con el guion 60→20 sí, y z final = 0,4264;
  - bajadas 0,10/0,20/0,30: llega a z = 0,0764, cabeceo mínimo ±1,5° de los valores de la tabla y velocidad de cabeceo ≤ 90°/s·1,01;
  - escaleras: los tres casos de 0,20/0,25.
- Rampa lisa (regla nueva): 30° sube y 45° no.
- Girar en el sitio con el costado a 5 mm de una pared de 0,5 m: |Δguiñada| < 1° y sin penetrar.
- Brazo contra pared: parado tocando con q = 80°, bajar a 20° hace retroceder la base 0,36·(cos 20° − cos 80°) = 0,2758 ± 2 mm.
- Regresiones de los fallos del prototipo:
  - canto con el CdG detrás: no se queda "en equilibrio" sin tracción;
  - final de bajada: x crece monótona, sin ciclo;
  - colocar mal apoyado no teletransporta más de empuje_max.
- Determinismo: un paso(dt) y dos paso(dt/2) dan lo mismo a ±1 mm y ±0,1°.

D) Rendimiento: script aparte, no en colcon test por el reloj del WSL (0,92x). Escena de escalera, 3000 ticks: media < 5 ms y p99 < 15 ms.

9. RENDIMIENTO
- Medido: 17,5 µs por línea (oruga analítica contra un perfil de 10 vértices, Python 3.14 en Windows).
- Una evaluación completa del robot, 13 líneas, cuesta ~0,23 ms.
- Un tick normal son 6 a 8 evaluaciones (~2 ms); uno de vuelco, ~15 (~4 ms).
- En WSL con Python 3.10, contar con 1,5 a 2 veces más. Presupuesto: 20 ms.

10. GAZEBO
No lo reabriría. Matiz: creo recordar que Gazebo Fortress, el de Humble, ya trae sistemas de orugas (TrackController/TrackedVehicle, de la SubT). No lo he comprobado; si es así, "no modela orugas" no es del todo exacto.

La razón de peso para no usarlo es otra: en Gazebo los brazos y la tracción los movería el simulador y no el firmware, y se perdería la paridad con HW_SIMULADO, que es justo lo que se quiere ensayar. Además habría que rehacer el modelo en SDF, ajustar fricciones y contactos, y el render y el tiempo real en WSLg son dudosos.