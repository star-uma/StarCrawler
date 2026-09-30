ESPECIFICACIÓN FINAL — MUNDO SIMULADO CON OBSTÁCULOS PARA STARCRAWLER (estilo RoboCup Rescue / German Open)
Rama feature/ros2, ROS 2 Humble, Python 3.10 (WSL Ubuntu 22.04). Sin Gazebo. No se ha tocado ningún fichero del repo.

Qué está verificado y qué no:
- Verificado solo en los prototipos del scratchpad (C:\Users\mario\AppData\Local\Temp\claude\C--Users-mario-Documents-TFM\084ea808-c449-41a1-a142-794f1198f2ca\scratchpad\):
  - arena\arena_proto.py y practica_rrl.yaml: el compilador de la arena;
  - proto_contacto2d.py, proto2.py, proto5.py, escenas5.py y esc_escalera.py: el contacto en 2D.
- He comprobado contra el código lo que doy por hecho aquí:
  - el parámetro publish_tf de odometry_node;
  - que la GUI mezcla /joint_states por nombre;
  - el bit 7 de error_bits en sim_core y en app.c;
  - que crawler_angle y /joint_states llevan el mismo valor;
  - que moverJunta de vista3d no recorta por límites;
  - que nadie lee el límite de chassis_lift_joint.
- Todo lo demás está solo diseñado.

======================================================================
§0 RESUMEN
======================================================================
Hay un nodo nuevo, mundo_node, en el paquete starcrawler_sim:
1. Carga un YAML de mundo.
2. Observa /starcrawler/state, lo publique starcrawler_sim o el ESP32 en HW_SIMULADO.
3. Calcula cuasiestáticamente cómo se apoya el robot en el terreno: sube, baja, se bloquea o vuelca.
4. Publica la pose VERDADERA:
   - la TF odom->base_footprint (x, y, rumbo);
   - las tres juntas virtuales del chasis: altura absoluta, cabeceo y balanceo.

En modo mundo cambian dos cosas del grafo:
- odometry_node sigue publicando /odom, que es la estimación, pero no su TF.
- chasis_node no se lanza.

El mundo se dibuja con un MarkerArray latcheado. RViz lo pinta tal cual y la web lo traduce.

Nada realimenta al robot: el firmware y sim_core no cambian. Por eso sim:=true y el ESP32 en HW_SIMULADO se comportan igual.

Grafo en modo mundo:
  mando (DS4 por UDP, joy o web) -> twist_mux / crawler_mux -> [starcrawler_sim | ESP32 HW_SIMULADO]
    -> /starcrawler/state -> mundo_node -> TF odom->base_footprint + /joint_states (chasis) + /mundo/*
    -> robot_state_publisher -> RViz (mundo.rviz) y web /3d
    -> odometry_node -> /odom sin TF: flecha gris en RViz y fantasma en la web; enseña la deriva.

======================================================================
§1 DECISIONES DE ARQUITECTURA Y CONTRADICCIONES RESUELTAS
======================================================================
1.1 Marco y dueño de la pose (la contradicción principal)
Los frentes proponían tres cosas:
- integración, map->odom;
- arena, una TF arena->odom;
- vistas, el mundo en odom.

ELEGIDO: el mundo vive en odom. Con mundo, mundo_node es el ÚNICO que publica odom->base_footprint y chassis_lift/pitch/roll_joint.

Motivos:
- (a) Una sola cadena de TF, con z, cabeceo y balanceo en UN mismo mensaje. Si la z fuera en una TF y el cabeceo en otra, habría un salto de un fotograma en los cantos.
- (b) plano.rviz y la web valen casi tal cual. La web ya mezcla /joint_states y solo cambia de dónde saca x, y y rumbo.
- (c) No hace falta un historial de sellos, ni un TransformListener en la GUI, y no hay riesgo de TF_REPEATED_DATA.
- (d) La deriva de la odometría se ve gratis: /odom se sigue dibujando y se adelanta al robot cuando este se bloquea.

La objeción de integración de que la odometría se volvería perfecta no aplica. odometry_node sigue integrando y publicando /odom; solo cambia a publish_tf:=false, un parámetro que ya existe.

Coste asumido: en modo mundo el marco odom es el del mundo (la verdad) y /odom es la estimación. Cuando haya EKF o IMU real, habrá que pasar a map->odom (fuera de alcance).

1.2 chasis_core no se toca
- Con mundo, chasis_node no se lanza: dos publicadores de las juntas del chasis harían parpadear el modelo en RViz y en la web.
- El robot real sigue con chasis_node en llano.

1.3 Nombres
- Tópicos /mundo/* (no /arena/*).
- Argumento mundo:=.
- Carpeta starcrawler_sim/mundos/.

1.4 Dónde va el código: todo el código nuevo va en starcrawler_sim.
- mundo_core.py: del fichero a sólidos, altura, perfil y geometría visual.
- contacto_core.py: apoyo y bloqueo.
- marcadores.py: indicadores y estado.
- mundo_node.py.

starcrawler_sim pasa a depender de starcrawler_odometry, del que usa chasis_core.Geometria, geometria_desde_urdf, odometry_core.velocidades_del_robot e integrar. No se mueve nada a starcrawler_common.

1.5 Geometría para las vistas
- Un MarkerArray latcheado, /mundo/marcadores.
- gui_node lo traduce con un módulo puro, mundo_modelo.py, como urdf_modelo.py hace con el URDF.
- La GUI no lee YAML ni depende de mundo_core.
- Se descarta /arena/descripcion.

1.6 Estado del terreno
- std_msgs/String con JSON (§6.5), no DiagnosticStatus.
- Ningún mensaje nuevo en starcrawler_msgs, porque firmware.sh lo copia al firmware en cuanto difiere.

1.7 Propuesta plana
- Sale de /starcrawler/state con odometry_core.velocidades_del_robot, la misma función y los mismos parámetros que la odometría. No sale de los incrementos de /odom: así el mundo no depende de que la odometría esté viva.
- El avance horizontal es v·f, con f = media de n_z de los contactos con tracción (1 en llano).
- No es v·cos(cabeceo): en llano, con los brazos asimétricos, el chasis cabecea y cos(cabeceo) rompería la paridad con /odom.

1.8 Paridad en llano
- "Idéntico a chasis_core a 1e-9" es imposible.
- Contrato: ±0,05 mm y ±0,01° en las poses por pares.
- Con UNA sola oruga bajada, el modelo nuevo apoya en tres contactos, que es más correcto, y difiere de chasis_core. Queda aceptado y documentado.

1.9 Robot real con mundo (bit 7 de error_bits a 0): MODO ESPEJO
- Borra el mundo.
- Sigue publicando la pose en llano, porque en modo mundo ni la odometría publica la TF ni hay chasis_node, y sin eso el modelo se rompería.
- Marca estado 'espejo' y avisa con ERROR una vez.
- Dura hasta reiniciar el nodo.

1.10 IMU simulada: fuera. No tiene consumidor, y una IMU inventada junto al robot real es peligrosa. La web saca el cabeceo y el balanceo de /mundo/estado.

1.11 Paleta
- Gris neutro según la altura, como proponía vistas; cada elemento puede llevar un 'color' opcional.
- Los colores NIST no van por defecto: se comerían el ámbar de la oruga RL.

1.12 Rastro con z: no hay Path nuevo. RViz dibuja /mundo/verdad con un display Odometry; la web usa z_suelo de /mundo/estado.

1.13 Indicadores
- Van en otro MarkerArray, /mundo/indicadores.
- Todos en marco odom, sin frame_locked.
- La web los traduce con el mismo módulo que el mundo.

1.14 Recarga del YAML al guardarlo: se mira el mtime cada 1 s. El único servicio es el de reiniciar.

1.15 Primitivas: se recortan a las robustas (§3.4). Quedan fuera tubos y cilindros, zanjas, 'encima', campos girados y techos no planos.

1.16 Gazebo: sigue descartado (§14).

======================================================================
§2 FICHEROS
======================================================================
Nuevos:
- ros2_ws/src/starcrawler_sim/starcrawler_sim/:
  - mundo_core.py (puro)
  - mundo_check.py (CLI puro)
  - contacto_core.py (puro)
  - marcadores.py (puro)
  - mundo_node.py
- ros2_ws/src/starcrawler_sim/mundos/: llano.yaml, escalon.yaml, rampa.yaml, escalera.yaml, practica_rrl.yaml
- ros2_ws/src/starcrawler_sim/test/:
  - test_mundo_core.py
  - terreno_prueba.py
  - test_contacto_geometria.py, test_contacto_reposo.py, test_contacto_avance.py
  - test_marcadores.py
  - test_mundo_integracion.py
  - medir_contacto.py (no empieza por test_, así que pytest no lo recoge)
- ros2_ws/src/starcrawler_description/rviz/mundo.rviz
- ros2_ws/src/starcrawler_gui/starcrawler_gui/mundo_modelo.py (puro) y ros2_ws/src/starcrawler_gui/test/test_mundo_modelo.py

Modificados:
- starcrawler_sim: package.xml y setup.py.
- starcrawler_bringup/launch: robot.launch.py y rviz.launch.py.
- starcrawler_description/urdf/starcrawler.urdf.xacro: solo el límite de chassis_lift_joint y una línea del comentario.
- starcrawler_gui: gui_node.py, servidor.py, vista3d.py, package.xml, test/test_servidor.py y test/test_vista3d.py.

NO se tocan:
- el firmware y micro_ros_esp32_apps;
- sim_core.py y sim_node.py;
- chasis_core.py y chasis_node.py;
- odometry_core.py y odometry_node.py;
- plano.rviz, starcrawler_msgs y teleop.

Reglas:
- Los módulos puros no importan rclpy.
- Todo compatible con Python 3.10, sin numpy.
- Comentarios escuetos: el razonamiento va a los .md.
- Commits de asunto más dos o tres líneas, uno por causa y SIN línea de coautor (CLAUDE.md del repo).

======================================================================
§3 FICHERO DE MUNDO (YAML, formato 1)
======================================================================
3.1 Nivel superior
- formato: 1 (obligatorio).
- nombre y descripcion.
- inicio: {x, y, rumbo}. Es la pose de salida en coordenadas del fichero (m y grados) y se convierte en el ORIGEN DE ODOM.
- limites: [[x0, y0], [x1, y1]] (opcional; solo para validar).
- paredes: [{puntos: [[x, y], ...], alto: 0.8, grosor: 0.02}] (opcional).
- zonas: [{nombre, de: [x, y], a: [x, y]}] (opcional; solo pintura y rótulo).
- carriles: [...] (opcional).
- elementos: [...] (opcional). Elementos sueltos en coordenadas del fichero, con marco en el origen (0, 0), rumbo 0 y 'ancho' obligatorio.

Una clave desconocida es un error: así no hay erratas silenciosas.

3.2 Carril
- Claves: nombre, titulo, origen [x, y] (centro del borde de entrada), rumbo (grados), largo, ancho (el de sus elementos por defecto), color (opcional) y elementos.
- Marco del carril: s a lo largo, en el sentido de avance; t lateral, positivo a la izquierda.

3.3 Claves comunes a todos los elementos
- tipo.
- pos: [s, t], el centro del borde de ENTRADA; el elemento se extiende hacia +s.
- pos: [sigue, t]: empieza donde acabó el anterior del mismo carril. Es un error si es el primero.
- rumbo: grados, relativo al carril; solo en caja, rampa y escalera.
- ancho: por defecto, el del carril.
- z: cota de la base; solo en rampa y escalera, 0 por defecto.
- color: [r, g, b], opcional.

3.4 Primitivas del formato 1
- caja: largo y alto. Para escalones, palés, plataformas y vallas.
- rampa:
  - exactamente dos de {largo, alto, pendiente (°)};
  - alto con signo: negativo es una bajada desde z;
  - sin alto, alto = +largo·tan(pendiente);
  - techo plano de z a z + alto; error si la cota final queda por debajo de 0.
- escalera:
  - claves: peldanos n (contrahuellas hasta el rellano), contrahuella h, huella o pendiente (huella = h/tan(pendiente)), rellano (largo; una huella por defecto) y bajada opcional {huella | pendiente};
  - genera cajas: n−1 peldaños de subida a z+h, ..., z+(n−1)h; el rellano a z+n·h; y, con bajada, n−1 peldaños de bajada en espejo;
  - sin bajada, tras el rellano queda un cortado de n·h (mundo_check avisa).
- campo_escalones:
  - claves: celda (0,10), unidad (0,10), base (0), y alturas o patron;
  - alturas es una matriz de enteros ≥ 0: la fila va a lo largo desde la entrada y la columna de izquierda (t = +ancho/2) a derecha;
  - patron puede ser plano_cruz o colina_diagonal, con filas y columnas. Son los del prototipo arena_proto.py, inspirados en ASTM E2828; no son la norma;
  - un poste por celda, de altura base + unidad·a; con a = 0 no hay poste;
  - mide filas·celda por columnas·celda (ignora el ancho). No admite rumbo.
- campo_rampas:
  - claves: celda (0,6), pendiente (15), y orientaciones o patron;
  - orientaciones es una matriz de letras que dice hacia dónde SUBE cada celda: A = +s, T = −s, I = +t, D = −t;
  - patron: cruzadas (A si i+j es par, I si es impar) o continuas (A en las filas pares, T en las impares), con filas y columnas;
  - cada celda es una rampa de 0 a celda·tan(pendiente). No admite rumbo.
- viga:
  - de: [s, t] y a: [s, t], en el marco del carril (no usa pos);
  - seccion (0,10, el ancho de la viga) y alto (por defecto igual a la sección);
  - es una caja orientada de 'de' a 'a' y no se recorta al carril;
  - sirve para bordillos, K-rails y barreras.

Fuera del formato 1: tubo, zanja/hueco, encima y rumbo en los campos. Dan el error "tipo X no soportado en formato 1".

3.5 Compilación
- Todo se reduce a SÓLIDOS: un rectángulo orientado (centro, rumbo, largo, ancho) con techo PLANO z = a + bx·x + by·y, macizo hasta z = 0, de clase terreno o pared.
- Cada segmento de pared es un sólido de largo = segmento + grosor, para que las esquinas cierren.
- Coordenadas: del fichero al carril, p = origen + R(rumbo)·(s, t). Después, a odom: p_odom = R(−inicio.rumbo)·(p − (inicio.x, inicio.y)), y los rumbos restan inicio.rumbo. TODO lo que sale de mundo_core está en odom.
- altura(x, y) = max(0, techos de los sólidos que contienen el punto), bordes incluidos. Las paredes cuentan: para el contacto son sólidos altos cuyas caras dan contactos de pared.
- Errores: ErrorMundo(ValueError) con un mensaje que dice dónde está, del tipo "carril_3, elemento 2 (viga): falta 'a'". Se comprueban tipos, claves, números finitos, que largo, ancho y alto sean > 0, cotas ≥ 0 y matrices rectangulares.
- Índice espacial: una rejilla de 0,25 m que guarda qué sólidos caen en cada celda.

======================================================================
§4 mundo_core.py (Parte A)
======================================================================
API:
  class ErrorMundo(ValueError)
  @dataclass(frozen=True) class Solido:
      cx, cy, rumbo, largo, ancho, a, bx, by, clase, color, origen
  class Mundo:
      nombre, inicio, solidos
      zonas, carriles            # en odom
      altura(x, y) -> float
      perfil(x0, y0, ux, uy, s0, s1) -> List[Tuple[float, float]]
      visual() -> List[dict]
      resumen() -> str
  def cargar(texto_yaml: str, nombre_fichero: str = '') -> Mundo
  def cargar_fichero(ruta: str) -> Mundo
  def resolver_ruta(nombre_o_ruta: str) -> str

resolver_ruta: un nombre como 'escalon' se busca en share/starcrawler_sim/mundos/escalon.yaml, importando ament_index_python solo al llamarla. Lo que acaba en .yaml o tiene '/' se toma como ruta tal cual.

4.1 perfil(), EXACTO
- La recta es P(s) = (x0, y0) + s·(ux, uy), con (ux, uy) unitario y s en [s0, s1].
- Devuelve la polilínea (s, z) del terreno a lo largo de la recta:
  - s no decrece; el primer vértice está en s0 y el último en s1;
  - cada salto vertical son dos vértices seguidos con la misma s: primero el valor por la izquierda y después el de la derecha.
- Algoritmo:
  1. Candidatos: los sólidos de las celdas que recorre el segmento (DDA, sin duplicados).
  2. Cada candidato se recorta con la recta en su marco local (slabs). Sale un intervalo [sa, sb] ∩ [s0, s1] en el que z(s) es lineal, porque el techo es plano.
  3. Se añade el suelo, z = 0 en [s0, s1].
  4. Envolvente superior:
     - los puntos de ruptura son los extremos de los intervalos y los cruces entre pares de rectas activas;
     - en cada ruptura se evalúa el valor por la izquierda (activos en (s−ε, s]) y por la derecha (activos en [s, s+ε));
     - si difieren más de 1e-9, se ponen dos vértices;
     - se pueden fusionar los vértices colineales.
- Objetivo de rendimiento en Python 3.10: ≤ 50 µs por recta de 2 m fuera de los campos y ≤ 150 µs dentro de un campo de escalones. Si no llega, se puede compilar el campo como un único sólido 'rejilla', pero solo si se mide que hace falta.

4.2 altura(x, y): con el índice espacial, ≤ 10 µs.

4.3 visual()
Devuelve una lista de dicts planos. Es el contrato COMÚN que mundo_node convierte a Marker (§6.6):
  {ns, id,
   tipo: 'triangulos' | 'lineas' | 'tira' | 'esferas' | 'flecha' | 'texto',
   puntos: [(x, y, z), ...],
   colores: [(r, g, b, a), ...] | None,
   color: (r, g, b, a),
   escala: float,          # grosor de línea, diámetro o alto del texto
   texto: str | None,
   pos: (x, y, z) | None}

Contenido:
- ns 'terreno', id 0, triangulos:
  - todos los sólidos de clase terreno: la cara de arriba (2 triángulos) y las 4 laterales hasta z = 0, estas solo si el techo pasa de 1 mm;
  - orden ANTIHORARIO visto desde fuera: rviz2 en Humble calcula la normal como (p1−p0)×(p2−p0) y no descarta caras, así que con el orden al revés la cara sale oscura sin dar error;
  - color por vértice: el 'color' del elemento si lo tiene; si no, gris según la altura del vértice, interpolando de #6b675f en z = 0 a #cdc6b6 en z ≥ 1,0 m. Alfa 1.
- ns 'paredes', id 0, triangulos: color (0,478, 0,510, 0,549, 0,28).
- ns 'aristas', id 0, lineas:
  - las 4 aristas de arriba y las 4 verticales de cada sólido de terreno con techo > 1 mm;
  - color (0,11, 0,11, 0,10, 0,7), escala 0,004;
  - sin aristas, los escalones no se distinguen con la luz de RViz.
- ns 'zonas', id k, tira: el contorno cerrado de cada zona a z = 0,002, blanco con alfa 0,5, escala 0,01.
- ns 'etiquetas', id k, texto:
  - el título de cada carril, en su origen, a la altura máxima del carril + 0,4;
  - el nombre de cada zona, en su centro, a 0,3;
  - escala 0,08 y color (0,76, 0,76, 0,72, 1).

Aviso, no error, si pasa de 60 000 vértices.

4.4 mundo_check.py: main(argv) -> int
- Se usa así: ros2 run starcrawler_sim mundo_check practica_rrl.
- Carga el mundo e imprime el resumen: sólidos, triángulos, altura máxima y, por carril, el tramo de s ocupado y su altura máxima.
- AVISA de:
  - saltos verticales de más de 0,40 m, por encima del máximo de primer contacto (0,4017);
  - más de 2 unidades de salto entre celdas vecinas de un campo (regla 1 de Jacoff et al.);
  - el cortado al final de una escalera sin bajada;
  - elementos fuera de 'limites';
  - solapes entre elementos del mismo carril.
- Sale con código 1 si hay un ErrorMundo.

======================================================================
§5 contacto_core.py (Parte B)
======================================================================
Es un modelo cuasiestático, sin dinámica.
- Validado en 2D (vista lateral) con los prototipos del scratchpad:
  - proto_contacto2d.py: la geometría y la ley del primer contacto;
  - proto2.py: el plano de apoyo;
  - proto5.py: el vuelco hasta el primer contacto nuevo; es la versión buena;
  - escenas5.py y esc_escalera.py: las escenas.
- Lo 3D (balanceo, líneas, triángulo de apoyo) NO está prototipado.

5.1 Geometría
Se lee del URDF. Los tests usan una geometría fija, GEO_PRUEBA, con estos mismos números.
- Orugas, chasis_core.Geometria:
  - pivotes en (±0,30, ±0,24);
  - polea activa R1 = 0,0764 en el pivote y pasiva R2 = 0,0415 en la punta, con L = 0,36 entre ejes;
  - altura_reposo 0,0764;
  - β = asin((R1−R2)/L) = 5,563°, el belt_angle del xacro.
- Forma de la oruga i en el plano x-z del cuerpo: la envolvente convexa del círculo (P_i, R1) y del círculo (T_i, R2), con T_i = P_i + L·(σ_i·cos q_i, sin q_i):
  - σ = +1 en las delanteras y −1 en las traseras;
  - q es la elevación, positiva hacia arriba;
  - la envolvente son dos arcos y dos tangentes exteriores.
- Chasis: el rectángulo x en [−0,30, 0,30], z en [−0,05, 0,05]. base_link está en el plano de los pivotes; la panza queda a 0,0264 del suelo en reposo.
- Masas:
  - chasis, 20 kg en (0, 0, 0);
  - cada brazo, 3 kg en P_i + (L/2)·(σ·cos q, 0, sin q);
  - el CdG es la media ponderada.
- geometria_robot_desde_urdf(xml) lee:
  - lo de chasis_core.geometria_desde_urdf;
  - la caja visual de base_link: 0,60 x 0,40 x 0,10;
  - el ancho de oruga, del length de los cilindros: 0,08;
  - las masas de los <inertial>: base_link 20 kg; crawler_*_link 3 kg, con su origin a L/2.

5.2 Pose y convenios
- Pose3D(x, y, yaw, z, cabeceo, balanceo).
- (x, y, yaw) son los de base_footprint en odom; z es la altura de base_link (los pivotes) sobre z = 0.
- R = Rz(yaw)·Ry(cabeceo)·Rx(balanceo), el orden de las juntas virtuales.
- REP-103, como en chasis_core: cabeceo positivo = morro abajo; balanceo positivo = lado izquierdo arriba.
- En llano y en reposo, z = 0,0764.

5.3 Líneas: el 3D se trata como cortes verticales paralelos al rumbo
- Posiciones laterales en el cuerpo:
  - orugas derechas en l = −0,275 y −0,205; izquierdas en +0,205 y +0,275 (±0,24 ± 0,035). Cada línea la comparten la oruga delantera y la trasera de ese lado;
  - chasis en l = −0,19, −0,095, 0, +0,095 y +0,19;
  - en total, 9 perfiles por pose.
- Cada línea en el mundo: base = (x, y) + l·(−sin yaw, cos yaw) y dirección u = (cos yaw, sin yaw). Su perfil es terreno.perfil(base, u, −1,0, +1,0).
- Un punto (bx, bz) de una pieza, en el cuerpo, pasa al plano de la línea así:
    s = cosθ·bx + sinθ·bz + sinθ·sinφ·l
    z = z_base − sinθ·bx + cosθ·bz + cosθ·sinφ·l
  con θ el cabeceo y φ el balanceo.
- Aproximación: se desprecian el factor cosφ sobre bz y el desplazamiento lateral por el balanceo. El error llega a unos 4 cm con la punta bajada a −45° y 10° de balanceo.
- Caché de perfiles: mientras la base solo avance a lo largo del rumbo (w = 0), los perfiles valen desplazando s. Se recalculan cuando:
  - el yaw cambia más de 0,05°;
  - hay un desplazamiento lateral de más de 0,5 mm;
  - la base se aleja más de 0,15 m del punto donde se consultaron.

5.4 Contacto en una línea
Para cada pieza (oruga o chasis) y cada rasgo del perfil se calcula una RESTRICCIÓN: la z_base mínima para que la pieza no quede por debajo del perfil, su punto de contacto y su normal terreno->robot (en el plano de la línea; en 3D, con componente lateral 0).
- Círculo de centro relativo c y radio r contra un tramo no vertical de pendiente α, si la proyección de c cae dentro: z_c ≥ z_tramo(c_s) + r/cos α. Normal: la del tramo.
- Círculo contra un vértice v del perfil (incluidos los extremos de los saltos), si |c_s − v_s| < r: z_c ≥ v_z + sqrt(r² − (c_s − v_s)²). Normal: (c − v)/|c − v|.
- Tangente (segmento) contra un vértice dentro de su rango en s: z_base ≥ v_z − z_seg(v_s). Normal: la del segmento, hacia el robot.
- Chasis: sus 4 aristas contra los vértices y sus 4 esquinas contra los tramos, de la misma manera.

Clasificación, según el ángulo de ataque φa entre la normal y la vertical:
- APOYO si φa ≤ 75° en una banda o φa ≤ 60° en el chasis.
- PARED en otro caso. La cara de un escalón contra la polea pasiva da 90°; un canto por encima del centro de un círculo, más de 90°.
- TRACCIÓN: solo la dan las bandas. En vértices (cantos), siempre; en tramos, solo si su pendiente es ≤ 40°. El chasis nunca tracciona.

Candidatos del apoyo:
- todas las restricciones de APOYO a menos de 6 cm de la máxima, 10 como mucho;
- cada una lleva su H_k (la z_base requerida), su posición en planta (s_k a lo largo, l_k lateral) y su pieza.

Holgura de la oruga i = z − max(H_k de sus restricciones de apoyo); es ≥ 0. La oruga "apoya" si la holgura es ≤ 0,5 mm.

5.5 Reposo: z, cabeceo y balanceo con (x, y, yaw) fijos
Es el "plano de apoyo con brazos de palanca": generaliza las diagonales de chasis_core.
- Plano de apoyo:
  - g es la proyección en planta del CdG;
  - se busca el plano P(p) = a + b·(p − g) que cumple P(p_k) ≥ H_k en todos los candidatos y deja P(g) mínimo. Es un programa lineal de 3 variables;
  - el óptimo es un triángulo de candidatos que contiene a g, o una arista si g cae sobre ella;
  - primero se prueba el soporte del tick anterior; si no vale, se enumeran ternas;
  - si hay dos caras óptimas empatadas, se promedian. En llano, con los cuatro pivotes y g en el centro, esto es exactamente la regla de las diagonales.
- Corrección de actitud:
  - cabeceo −= atan(b_s) y balanceo += atan(b_l);
  - relajación 0,5, como mucho 5° por iteración, 8 iteraciones por tick y arranque en caliente.
- Al final, SIEMPRE z = max(H_k de APOYO): nunca penetra en vertical, aunque no haya convergido.
- Si z tiene que bajar, baja como mucho 1 m/s·dt.
- VUELCO. Se da si g queda fuera del polígono de apoyo, o a menos de 1 mm de su borde sin nada más allá. El polígono es la envolvente convexa en planta de los contactos activos (H_k ≥ z − 0,5 mm). Entonces:
  - el sólido gira alrededor de la arista que cruza g o, con un único contacto, alrededor de un eje horizontal que pasa por él, perpendicular a g − p;
  - el pivote queda fijo en el mundo; se mueven x, y, z, cabeceo, balanceo y un poco el yaw (se recompone con ZYX);
  - gira en el sentido que baja el CdG, como mucho 90°/s·dt (1,8° por tick a 50 Hz);
  - se para en el primer contacto nuevo, que se busca por bisección en el ángulo (10 mitades). Contacto nuevo = alguna restricción con H > z + 0,1 mm;
  - cayendo = true mientras gira o cae.
- VOLCADO: si |cabeceo| o |balanceo| pasa de 75°, se congela la pose con la bandera puesta hasta colocar().

5.6 Avance y bloqueo, en cada tick
(a) Subpasos:
  - n = ceil(max(|v|·dt, 0,75·|w|·dt, 0,40·max|Δq|) / 0,002): nada se mueve más de 2 mm por subpaso;
  - lo normal es n = 1; con w = 0,2 rad/s, n = 2;
  - las elevaciones se interpolan entre las del tick anterior y las nuevas.
(b) Tracción:
  - hace falta alguna restricción de banda en el apoyo activo que tenga tracción y holgura ≤ 2 mm;
  - sin ella, ese subpaso no hay avance ni giro (sin_traccion). El caso típico es la panza sobre un canto con las orugas al aire;
  - los brazos se mueven siempre.
(c) Propuesta plana:
  - odometry_core.integrar(x, y, yaw, v·f, w, dt_sub);
  - f = media de n_z (la componente vertical de la normal en el plano de la línea) de las restricciones con tracción: 1 en llano y cos 15° en una rampa de 15°.
(d) Guardia contra el teletransporte:
  - si la z requerida de una pieza en una línea sube más de d·tan 75° + 2 mm, se deshace la parte plana del subpaso por bisección (6 mitades) y se marca bloqueado con esa pieza;
  - d es lo que se ha movido esa pieza en planta, sumando L·|Δq| en las orugas;
  - así se pilla entrar de lado en una pared al girar. Bajar un brazo sobre algo está siempre permitido.
(e) Empuje:
  - las restricciones de PARED que penetran (H > z + 0,1 mm) se resuelven en horizontal: la base se mueve a lo largo del rumbo, en el sentido de su normal, 5 mm como mucho por subpaso;
  - sin ese tope, en el prototipo colocar un robot mal apoyado lo teletransportaba 0,49 m;
  - se hace antes y después del reposo;
  - si hay empujes en sentidos opuestos, se deshace el subpaso; si aun así penetra, atrapado = true.
(f) bloqueado: el empuje o la guardia se han comido más de la mitad del avance propuesto. Se devuelve la pieza: 'FR', 'FL', 'RR', 'RL' o 'chasis'.

El mundo es SOLO observador: nunca frena ni los brazos ni la tracción.
- Contra una pared, las orugas patinan y /odom sigue avanzando.
- Un brazo que baja contra una cara empuja la base hacia atrás.

Ley del primer contacto (chasis en llano, φb = 75°), para los tests:
- si q + β ≤ φb: h_max(q) = R1 + L·sin q − R2·cos φb;
- si no: R1·(1 − cos φb) = 0,0566;
- con las puntas en el suelo: R2·(1 − cos φb) = 0,031.

| q | h_max (m) |
|---|---|
| 0° | 0,0657 |
| 10° | 0,1282 |
| 20° | 0,1888 |
| 30° | 0,2457 |
| 45° | 0,3202 |
| 60° | 0,3774 |
| 69,4° | 0,4017 (el máximo) |
| ≥ 70° | 0,0566 |

Al bloquearse, la base se para a x_cara − (0,30 + 0,36·cos q + 0,0415): a 0,7015 de la cara con q = 0, a 0,6798 con 20° y a 0,6533 con 30°.

5.7 colocar(x, y, yaw, elevaciones, ...)
- Empieza con cabeceo = balanceo = 0 y z = max H.
- Hace el reposo sin límites por tick (hasta 50 iteraciones) y el vuelco sin límite de ángulo, girando sobre base_link: no mueve x, y ni yaw.
- El empuje queda acotado a 5 mm en total.
- Limpia la bandera de volcado.

5.8 Regresiones de los dos fallos del prototipo
1. Minimizar la altura del CdG con la planta fija deja resbalar el contacto: el robot quedaba "en equilibrio" sobre la panza, con el CdG 1,6 cm por detrás del canto y sin tracción. Por eso los brazos de palanca se fijan en cada iteración.
2. El vuelco con paso fijo de 0,5° se pasaba y hacía un ciclo de avance y retroceso al final de una bajada. Por eso se gira sobre el pivote hasta el primer contacto nuevo. Si se gira sobre base_link, el equilibrio inestable se vuelve neutro y el robot se queda en el canto.

5.9 API (pura)
  PIEZAS = ('FR', 'FL', 'RR', 'RL', 'chasis')

  class Terreno(Protocol):
      def perfil(self, x0, y0, ux, uy, s0, s1) -> Sequence[Tuple[float, float]]: ...

  @dataclass(frozen=True) class GeometriaRobot:
      orugas: chasis_core.Geometria
      chasis: Tuple[float, float, float]    # largo, ancho, alto
      ancho_oruga: float
      masa_chasis: float
      masa_brazo: float

  def geometria_robot_desde_urdf(xml: str) -> GeometriaRobot

  @dataclass(frozen=True) class Parametros:
      ataque_banda = rad(75); ataque_casco = rad(60); traccion_lisa = rad(40)
      caida_ang = rad(90)     # rad/s
      caida_lin = 1.0         # m/s
      vuelco = rad(75)
      subpaso = 0.002; empuje_max = 0.005
      tol_apoyo = 0.0005; tol_traccion = 0.002; tol_penetra = 0.0001
      eps_equilibrio = 0.001
      lineas_oruga = (-0.035, 0.035)
      lineas_chasis = (-0.19, -0.095, 0.0, 0.095, 0.19)
      iteraciones = 8; max_giro_iter = rad(5); relajacion = 0.5
      ventana_candidatos = 0.06; max_candidatos = 10

  @dataclass(frozen=True) class Pose3D: x, y, yaw, z, cabeceo, balanceo

  @dataclass(frozen=True) class Contacto:
      pieza, x, y, z
      normal: Tuple[float, float, float]    # en el mundo
      ataque
      tipo: 'apoyo' | 'pared'
      traccion: bool
      holgura

  @dataclass(frozen=True) class Estado:
      pose; elevaciones (4); contactos (tupla)
      apoya (4 bool); holguras (4); panza: bool
      bloqueado: Optional[str]
      sin_traccion; cayendo; volcado; atrapado
      cdg: (x, y, z)
      poligono: tupla de (x, y, z), antihoraria en planta
      margen    # distancia con signo de g al borde del polígono; + = dentro
      avance    # m en planta en este tick
      propuesto
      soporte, cache    # opacos

  def colocar(x, y, yaw, elevaciones, terreno, geo, par=Parametros()) -> Estado
  def paso(estado, v, w, elevaciones, dt, terreno, geo, par=Parametros()) -> Estado
      # v en m/s y w en rad/s
  def altura_trepable(q, geo, par=Parametros()) -> float   # la ley de §5.6
  def restricciones_linea(...) -> list                    # expuesta para los tests

5.10 Rendimiento
- Medido en el prototipo (Python 3.14 en Windows): 17,5 µs por línea contra un perfil de 10 vértices. Un tick normal son 6 a 8 evaluaciones, unos 2 ms.
- Objetivo en WSL con Python 3.10: media < 5 ms y p99 < 15 ms por tick en la escena de la escalera.
- Se mide con medir_contacto.py, no en colcon test: el reloj del WSL va a 0,92x.
- Si no se llega, se baja rate_hz a 25.

======================================================================
§6 mundo_node.py (Parte C)
======================================================================
6.1 Nodo 'starcrawler_mundo'

Suscribe:
- /starcrawler/state: RobotState, con qos_profile_sensor_data (el ESP32 publica best effort).
- /robot_description: String, reliable y transient local, depth 1.
- /initialpose: PoseWithCovarianceStamped, reliable, depth 1. El frame_id tiene que ser 'odom'; si no, se ignora con WARN.

Publica:
- TF odom->base_footprint, por TransformBroadcaster, a 50 Hz.
- /joint_states: JointState, depth 10, como chasis_node. Lleva:
  - chassis_lift_joint = z − altura_reposo;
  - chassis_pitch_joint = cabeceo;
  - chassis_roll_joint = balanceo;
  - el MISMO sello que la TF.
- /mundo/verdad: nav_msgs/Odometry, reliable, depth 10, a 50 Hz.
  - frame odom, child base_link;
  - la pose 6D de base_link;
  - twist.linear.x = avance real y angular.z = w.
- /mundo/marcadores: MarkerArray, reliable y transient local, depth 1.
  - se publica al cargar y al recargar: primero un DELETEALL y después lo que devuelve mundo_core.visual().
- /mundo/indicadores: MarkerArray, reliable y volatile, depth 5, a 10 Hz.
  - lifetime de 0,5 s; primero un DELETEALL.
- /mundo/estado: std_msgs/String con JSON, reliable, depth 10, a 10 Hz.

Servicio /mundo/reiniciar (std_srvs/Trigger): colocar() en el origen de odom, la salida del mundo, con las elevaciones actuales.

6.2 Parámetros
- mundo: nombre o ruta, obligatorio.
- rate_hz: 50.0.
- wheel_radius: 0.0764 y track_separation: 0.524. El launch los copia de odometry.yaml.
- timeout_estado_s: 0.5.
- recargar_auto: true.
- marco_odom: 'odom' y marco_base: 'base_footprint'.

Si el YAML no es válido al arrancar, el nodo no se cae: carga un mundo vacío y lo dice en el log y en el motivo del estado. Con recargar_auto, en cuanto se arregla el fichero, lo carga.

6.3 El tick (un temporizador a rate_hz)
- dt = tiempo desde el tick anterior, recortado a [0, 0,1] s.
- Elevaciones: el crawler_angle del último estado, que ya viene en radianes de elevación (no usar angulos.py). Donde encoder_ok es falso, se usa la última buena.
- v, w = odometry_core.velocidades_del_robot(track_speed_left, track_speed_right, wheel_radius, track_separation). Valen 0 si el estado tiene más de timeout_estado_s.
- estado = contacto_core.paso(...).
- Publica la TF, las juntas y la verdad; cada 5 ticks, además, el estado y los indicadores.
- Mide lo que tarda el paso (campo ms) y da un WARN limitado si pasa de 15 ms.
- Patinado acumulado: patinado += max(0, propuesto − avance).
- Antes de tener /robot_description: la TF en el origen y las juntas a 0, como cb_reposo de chasis_node.
- Sin estado del robot: elevaciones a 0 y el robot en la salida.

6.4 Compuerta del bit 7 (hardware simulado)
Si llega un RobotState con (error_bits >> 7) & 1 == 0, el nodo entra en MODO ESPEJO de forma permanente hasta reiniciarlo:
- cambia el mundo por uno vacío;
- publica un DELETEALL en /mundo/marcadores;
- pone modo = 'espejo';
- da una vez el ERROR "robot real o driver serie: el mundo se desactiva; el robot se dibuja en llano".
Cubre el robot real y simulate:=true (el driver serie no pone el bit 7) sin romper las vistas.

6.5 /mundo/estado (JSON, versión 1)
  {"v": 1, "mundo": "escalon", "version": n, "modo": "normal|espejo",
   "estado": "libre|bloqueado|sin_traccion|cayendo|volcado|atrapado",
   "pieza": "FR|FL|RR|RL|chasis" | null, "motivo": str, "consejo": str,
   "apoya": [b, b, b, b], "holgura": [m, m, m, m], "panza": b, "margen": m,
   "cabeceo": rad, "balanceo": rad, "altura": m, "z_suelo": m,
   "avance_orugas": m/s, "avance_real": m/s, "patinado": m, "ms": float}
- version sube en cada recarga.
- altura es la z de base_link; z_suelo, la altura() bajo base_footprint.

Prioridad del estado: volcado > atrapado > cayendo > bloqueado > sin_traccion > libre.

Textos de marcadores.estado_json, con las etiquetas del DS4 y del mando web:

| Estado | motivo | consejo |
|---|---|---|
| bloqueado por FR o FL | el morro choca con un canto de N cm | sube las delanteras (L1 · ▲ delanteras) |
| bloqueado por RR o RL | la cola choca | sube las traseras (R1 · ▲ traseras) |
| bloqueado por el chasis | el chasis choca | baja los cuatro brazos para levantar el chasis (L2 + R2) |
| sin_traccion | apoya la panza y las orugas no tocan | baja los brazos (L2 / R2) |
| volcado | más de 75° de inclinación | reinicia: /mundo/reiniciar o 2D Pose Estimate en RViz |
| atrapado | empujado por los dos lados | mueve los brazos o retrocede |
| espejo | sin bit 7: es el robot real o el driver serie | el mundo solo funciona con sim:=true o con el ESP32 en HW_SIMULADO |

N es la z del contacto que bloquea menos z_suelo, en cm.

6.6 Del dict al Marker (en mundo_node; común a visual() y a los indicadores)
- frame_id 'odom' y sello now.
- Pose identidad, salvo en 'texto', donde pose.position = pos.
- Tipos:
  - triangulos -> TRIANGLE_LIST (11);
  - lineas -> LINE_LIST (5);
  - tira -> LINE_STRIP (4);
  - esferas -> SPHERE_LIST (7);
  - flecha -> ARROW (0), con 2 puntos;
  - texto -> TEXT_VIEW_FACING (9).
- scale:
  - triangulos: (1, 1, 1);
  - lineas y tira: (escala, 0, 0);
  - esferas: (escala, escala, escala);
  - flecha: (0,02, 0,04, 0,05);
  - texto: z = escala.
- colors por vértice si hay colores.
- action ADD. lifetime 0 para el mundo y 0,5 s para los indicadores.

6.7 Indicadores: marcadores.indicadores(estado) -> list[dict]
Mismo contrato; cada ns con id 0:
- 'contactos': esferas de 0,035, blancas, en los contactos de apoyo activos de las bandas.
- 'panza': esferas de 0,035 en #fab219, en los contactos activos del chasis.
- 'poligono': tira cerrada de 0,012 por el polígono de apoyo, 5 mm por encima. Color según el margen: > 0,10 m, #c3c2b7; de 0,03 a 0,10, #fab219; < 0,03, #d03b3b.
- 'cdg': esferas (un solo punto) de 0,045, del color del polígono.
- 'plomada': lineas del CdG a la z del contacto más bajo.
- 'choque': flecha en #fab219 desde el punto que bloquea, 0,15 m en la dirección horizontal de su normal. Solo si está bloqueado.
- 'estado': texto a base_link + (0, 0, 0,30), escala 0,08; en ámbar #fab219, o en rojo #d03b3b si está volcado. Solo si no está libre.

6.8 Recarga
- Con recargar_auto, mira el mtime cada 1 s.
- Si el fichero cambia y es válido: sustituye el mundo, sube version, republica los marcadores y hace colocar() en la (x, y, yaw) actual.
- Si no es válido: conserva el anterior y saca el ErrorMundo por el log.
- Para editar en caliente, hay que lanzar con la ruta del fichero fuente (mundo:=/home/.../mundos/x.yaml). Lo instalado en share solo cambia con colcon build.

6.9 /initialpose
- Hace colocar(x, y, yaw) en odom.
- La odometría NO se reinicia, así que /odom queda desplazado y se ve como deriva. Hay que documentarlo.

======================================================================
§7 LAUNCH, URDF Y EMPAQUETADO (Parte C)
======================================================================
robot.launch.py:
- Argumento nuevo:
  DeclareLaunchArgument('mundo', default_value='', description='Mundo con obstaculos para el robot simulado: nombre en starcrawler_sim/mundos o ruta a un .yaml. Vacio = como hasta ahora')
- con_mundo = PythonExpression(["'", mundo, "' != ''"]) y sin_mundo = PythonExpression(["'", mundo, "' == ''"]).
- odometry_node: añadir {'publish_tf': ParameterValue(sin_mundo, value_type=bool)} a sus parámetros.
- chasis_node: condition=IfCondition(sin_mundo).
- Nodo nuevo:
  - starcrawler_sim/mundo_node, con name 'starcrawler_mundo' y condition IfCondition(con_mundo);
  - parámetros [{'mundo': mundo, 'wheel_radius': ..., 'track_separation': ...}], leídos de odometry.yaml con una función como topes_del_mando().
- RViz: -d con mundo.rviz si hay mundo y plano.rviz si no (con un PythonExpression).
- En el docstring, dos líneas de uso nuevas.

Con mundo vacío, el grafo es exactamente el de hoy: lo comprueba la prueba 1 de §12.

rviz.launch.py: argumento mundo (por defecto 'false') que elige mundo.rviz.

xacro:
- chassis_lift_joint pasa a <limit lower="-0.6" upper="2.0">.
- En el comentario: "las publica chasis_node, o mundo_node con un mundo cargado (entonces la altura es la del terreno)".
- Nadie lee hoy ese límite: robot_state_publisher y moverJunta no recortan, y solo lo usa joint_state_publisher_gui en view_model.

starcrawler_sim/package.xml:
- <depend>: starcrawler_odometry, nav_msgs, std_msgs, std_srvs, visualization_msgs y tf2_ros.
- <exec_depend>: python3-yaml y ament_index_python.

starcrawler_sim/setup.py:
- entry points mundo_node y mundo_check;
- data_files ('share/starcrawler_sim/mundos', glob('mundos/*.yaml')).

======================================================================
§8 RVIZ: starcrawler_description/rviz/mundo.rviz (Parte C)
======================================================================
Es una copia de plano.rviz con:
- Fixed Frame odom; Grid de 0,5 m en odom; RobotModel.
- MarkerArray 'Mundo': /mundo/marcadores, Depth 1, Reliable y Transient Local. Sin Transient Local, un RViz lanzado después del mundo no ve nada.
- MarkerArray 'Apoyo': /mundo/indicadores, Depth 5, Volatile.
- Odometry 'Real': /mundo/verdad, azul 57;135;229, Keep 1000, tolerancia 0,05.
- Odometry 'Odometria': /odom, gris 137;135;129, Keep 300. Es lo que cree la odometría: al bloquearse el robot, se le adelanta.
- TF apagado.
- Herramienta SetInitialPose (2D Pose Estimate) en /initialpose.
- Vista Orbit con Target Frame base_link, que sigue también la altura; Distance 3, Pitch 0,45.
- Vistas guardadas:
  - 'Perfil': rviz_default_plugins/ThirdPersonFollower sobre base_footprint, yaw +90°, pitch 0,1, distancia 2,2. No está verificada en Humble: si no carga, se quita;
  - 'Cenital': TopDownOrtho.

plano.rviz no se toca.

======================================================================
§9 WEB /3d (Parte D)
======================================================================
9.1 mundo_modelo.py (puro, hermano de urdf_modelo.py)
- class Traductor:
  - aplicar(markers): acepta objetos con atributos (el Marker de ROS, o SimpleNamespace en los tests) y aplica en orden ADD/MODIFY, DELETE (su ns/id) y DELETEALL sobre un diccionario indexado por (ns, id);
  - a_json() -> {"version": n, "piezas": [...], "omitidos": k}.
- Cada pieza:
    {"clave": "ns/id", "tipo": "triangulos|lineas|tira|esferas|flecha|texto",
     "pos": [x, y, z], "quat": [x, y, z, w], "escala": [sx, sy, sz],
     "color": [r, g, b, a], "puntos": [x0, y0, z0, x1, ...],
     "colores": [r, g, b, a, ...] | null, "texto": str}
  - puntos, redondeados al mm;
  - colores, a 3 decimales.
- Tipos aceptados: 11, 5, 4, 7, 0 (con 2 puntos) y 9. Cualquier otro, o un frame_id distinto de marco_fijo, suma en omitidos.
- traducir_lista(markers): para los indicadores; devuelve la lista de piezas, sin guardar estado.

9.2 gui_node.py
Parámetro nuevo marco_fijo = 'odom'. Suscripciones nuevas; ninguna pasa por registrar(), para que no cuenten como paquetes ni como enlace:
- /mundo/marcadores, reliable y transient local, depth 1:
  - pasa por el Traductor;
  - MUNDO['json'] = json.dumps(...) y MUNDO['version'] += 1;
  - ESTADO['mundo_v'] = versión.
- /mundo/indicadores, reliable, depth 5: ESTADO['marcas'] = traducir_lista(...).
- /mundo/estado:
  - ESTADO['terreno'] = el dict validado: 4 KB como mucho y solo claves conocidas;
  - lo que llegue mal se ignora con un WARN; no tumba el nodo.
- /mundo/verdad, qos_profile_sensor_data:
  - ESTADO['verdad'] = [x, y, yaw, v, w, z];
  - el yaw, con la misma fórmula ZYX de cb_odom.
- Un temporizador a 5 Hz pone terreno, marcas y verdad a None cuando llevan más de 0,5 s sin llegar.

Además:
- La geometría del mundo NUNCA va por /events.
- 'pose' sigue saliendo de /odom: el panel del mando lee pose[3] y pose[4].

9.3 servidor.py
- GET /mundo: MUNDO['json'] o, si no hay mundo, {"version": 0, "piezas": [], "omitidos": 0}.
- Siempre 200 y con Cache-Control no-store: no tener mundo es un estado normal.
- MUNDO = {'json': None, 'version': 0}, junto a MODELO.

9.4 vista3d.py
Solo cambia el <script type="module">; el mando no se toca.
- Carga reactiva:
  - en cada evento, si s.mundo_v ha cambiado, fetch('/mundo', {cache: 'no-store'});
  - se hace dispose del grupo y las etiquetas anteriores y se reconstruye.
- construirMarca(p), común al mundo y a los indicadores:
  - triangulos: BufferGeometry no indexada con computeVertexNormals(), DoubleSide, y vertexColors si hay colores (pasándolos de sRGB a lineal, como hace color());
  - lineas: LineSegments;
  - tira: Line;
  - esferas: InstancedMesh de esferas;
  - flecha: ArrowHelper;
  - texto: etiqueta HTML;
  - materiales MeshStandardMaterial con roughness 0,9 y metalness 0. Con alfa < 1: transparent, depthWrite false y renderOrder 1;
  - la cuaternión de ROS pasa tal cual, porque la escena ya va con Z arriba.
- Pose del robot:
  - objetivo = s.verdad || s.pose (x, y, yaw);
  - robot.raiz sigue en z = 0: la altura, el cabeceo y el balanceo llegan en las juntas del chasis, como hoy;
  - se mantienen el suavizado y el reinicio con saltos de más de 1 m.
- Indicadores: s.marcas se sustituye entero en cada evento, en un grupo aparte.
- Rastros:
  - el real, en --s1, con z = (s.terreno ? s.terreno.z_suelo : 0) + 0,004;
  - con verdad, además, el fantasma de /odom (s.pose) con LineDashedMaterial en --apagado;
  - "Borrar rastro" borra los dos.
- Cámara:
  - en modo seguir, centro.z = la z de base_link (verdad[5]), suavizada con tau de 0,3 s. Hoy está fija en 0,1;
  - botones nuevos 'Perfil' (az = yaw + π/2, el = 0,12) y 'Detrás' (az = yaw + π, el = 0,35), que siguen el rumbo mientras están activos.
- Sombras: solo con mundo, con un botón 'Sombras' y encendidas por defecto.
  - renderer.shadowMap con PCFSoft; mapa de 2048, o de 1024 si el ancho es menor de 760 px;
  - el sol sigue al robot: posición robot + (3, −2, 6), target el robot, cámara de sombra ortográfica de ±4 m;
  - el mundo proyecta y recibe sombras; el robot proyecta;
  - suelo ShadowMaterial de 20 x 20 m a z = 0,0005 con opacidad 0,35.
- Etiquetas:
  - capa #etiquetas dentro de #escena, con pointer-events none y z-index 1, por debajo de #mando, que está en 2;
  - en cada fotograma se proyecta un ancla; se ocultan detrás de la cámara o a más de 8 m;
  - estilo: 11 px, --tinta2, fondo rgba(13,13,13,.6) y radio 4 px;
  - SIEMPRE createElement + textContent: el texto viene de ROS, y test_el_mando_no_se_regenera cuenta los innerHTML;
  - botón 'Etiquetas', solo con mundo.
- Chip #terreno:
  - arriba al centro; en #escena.estrecho, abajo al centro, por encima de los controles. z-index 1;
  - texto #0d0d0d en negrita, como #sinEnlace, con el formato 'ESTADO · motivo · consejo';
  - ámbar #fab219 para bloqueado, sin_traccion, cayendo y atrapado; rojo #d03b3b para volcado;
  - gris para espejo, y para "terreno: sin datos" (hay mundo, pero el estado lleva más de 0,5 s sin llegar);
  - oculto cuando está libre.
- Aside:
  - sección 'Terreno', oculta sin mundo: estado, cabeceo y balanceo en grados (ámbar por encima de 25° y rojo por encima de 35°), altura del chasis, margen, avance real frente al de las orugas, patinado y error de odometría (la distancia entre verdad y pose);
  - 'Posición (odom)' pasa a 'Posición (real)' cuando hay verdad;
  - en #orugas, una columna 'apoya' ('●' en --tinta, o '○ 3,4 cm' en --apagado), dentro del innerHTML que ya existe.
- Rendimiento: con una malla por pieza basta, porque el mundo llega casi entero en la pieza 'terreno'. Si en el móvil baja de 30 fps, se apagan las sombras por defecto en pantallas estrechas.

9.5 package.xml de la GUI: <depend>visualization_msgs</depend>. En la web no entra ninguna dependencia nueva: todo es del núcleo de three.js 0.160.

======================================================================
§10 ARENAS DE EJEMPLO (Parte A), en starcrawler_sim/mundos/
======================================================================
- llano.yaml: formato 1, nombre llano, inicio (0, 0, 0) y sin nada. Es para la prueba de identidad.
- escalon.yaml: inicio (0, 0, 0) y una caja de 0,20 m de alto, 1,0 de largo y 1,2 de ancho, con pos [1.0, 0]:
  - la cara de entrada queda en x = 1,0;
  - se sube, se recorre la plataforma y se baja 20 cm en x = 2,0.
- rampa.yaml: ancho 1,2.
  - rampa de 15° hasta 0,20 m desde x = 1,0 (largo 0,746);
  - meseta de 0,8 m;
  - rampa de bajada de 30° (alto −0,20).
- escalera.yaml: escalera en x = 1,2, ancho 1,2:
  - peldanos 4, contrahuella 0,20, huella 0,25 (38,7°);
  - rellano 1,0, a 0,80 m;
  - bajada {huella: 0,25}.
- practica_rrl.yaml: la del prototipo (scratchpad\arena\practica_rrl.yaml), adaptada al formato 1.
  - Arena de 8 x 6 m.
  - Paredes de 0,8 m de alto y 0,02 de grosor: la exterior y tabiques en y = 1,2 (x de 0 a 6,4), y = 2,4 (de 1,6 a 8), y = 3,6 (de 0 a 6,4) e y = 4,8 (de 1,6 a 8).
  - Zonas:
    - salida [0-1,6] x [0-1,2];
    - bahía A [6,4-8] x [0-2,4];
    - bahía B [0-1,6] x [1,2-3,6];
    - bahía C [6,4-8] x [2,4-4,8];
    - meta [0-1,6] x [3,6-4,8].
  - inicio (0,80, 0,60, 0).
  - Carriles de 1,2 m:
    1. Origen (1,6, 0,6), rumbo 0:
       - el bordillo, una viga de 10 x 10 cm de (0,45, 0,6) a (0,45, −0,6);
       - rampa de 15° hasta 0,16 en s = 1,1; caja de 0,6 con 'sigue'; rampa de bajada con 'sigue' (z 0,16, alto −0,16, pendiente 15);
       - campo_rampas cruzadas de 2 x 2 celdas de 0,6 m a 15°, en s = 3,3.
    2. Origen (6,4, 1,8), rumbo 180:
       - cajas de 0,10 (s de 0,4 a 1,4) y de 0,20 (s de 2,0 a 3,0);
       - valla sin tubos: caja de 0,20 (s de 3,4 a 4,0) y caja de 0,40 (s de 4,0 a 4,6), con bajada de 40 cm.
    3. Origen (1,6, 3,0), rumbo 0:
       - K-rails: vigas de 10 cm de (0,3, 0,6) a (1,5, −0,6) y de 20 cm de (0,9, 0,6) a (2,1, −0,6);
       - escalera en s = 2,5: 3 contrahuellas de 0,20 a 35° (huella 0,286), rellano de 0,8 a 0,60 m y bajada a 45° (huella 0,20).
    4. Origen (6,4, 4,2), rumbo 180:
       - campo medio cúbico de 12 x 12 celdas de 0,10 con unidad 0,05 y patron plano_cruz, en s = 0,3;
       - campo cúbico de 12 x 12 celdas de 0,10 con unidad 0,10 y la matriz explícita de colina diagonal del prototipo, en s = 1,9;
       - sin la zanja, que queda fuera del formato 1.
    5. Origen (1,6, 5,4), rumbo 0, largo 6,4:
       - cajas de 0,15, 0,30 y 0,40 m de alto y 0,6 m de largo, en s = 0,4, 2,2 y 4,0;
       - la de 0,40 es el límite: queda por encima del primer contacto máximo (0,4017 m), así que casi seguro no sube. Sirve para verlo.
  - Se conservan los comentarios de fuente de cada elemento del prototipo (reglas RRL de 2019 a 2026D, Jacoff et al. 2008, NIST, ASTM, OARKit y Athena), separando las cotas citadas de las derivadas.
  - No he encontrado planos propios de la German Open: las cotas son las comunes de las reglas RRL y de NIST.

Cotas del robot que conviene tener a mano:
- 1,40 m de largo con los brazos en llano (0,75 m con los brazos a +90) y 0,56 m de ancho.
- Pivote a 7,6 cm del suelo.
- Panza a 2,6 cm; a 10 cm con los brazos a −17,5° y a 24,6 cm con los brazos a −45°.
- 0,053 m/s de avance y 4,69°/s de brazos.
- Gira sobre sí mismo en un círculo de 1,51 m con los brazos en llano y de 0,94 m con los brazos a +90. En los carriles de 1,2 m solo gira con los brazos arriba; en las bahías, con 0,79 m de holgura, también en llano.

======================================================================
§11 TESTS (pytest sin ROS, salvo donde se dice)
======================================================================
Parte A, test_mundo_core.py:
- Todas las mundos/*.yaml cargan (glob relativo al paquete).
- escalon:
  - altura(1,5, 0) = 0,20; altura(0,99, 0) = 0; altura(1,0, 0) = 0,20 (el borde es cerrado);
  - perfil((0, 0), (1, 0), 0, 2,5) = [(0, 0), (1,0, 0), (1,0, 0,2), (2,0, 0,2), (2,0, 0), (2,5, 0)].
- rampa: a media subida, 0,10.
- escalera: el peldaño k a k·0,20 y el rellano a 0,80.
- practica_rrl:
  - la transformación de inicio: el bordillo del carril 1 ocupa x de 1,20 a 1,30 en odom, con 0,10 de alto en |y| ≤ 0,6;
  - los carriles con rumbo 180 y 'sigue'.
- Campos: el número de postes, las alturas de plano_cruz, y que la celda A de las cruzadas está a media altura en su centro.
- Propiedad, con 500 rectas al azar en practica_rrl: la polilínea de perfil() evaluada en s al azar coincide con altura() a 1e-9 lejos de los saltos, y s nunca decrece.
- Errores con su mensaje: rampa con tres cotas, tipo desconocido, 'tubo no soportado en formato 1', clave desconocida y matriz no rectangular.
- Malla: normales hacia fuera (el producto escalar con el vector del centro del sólido al centro de la cara es positivo), colores en [0, 1], sin NaN e ids únicos por ns.
- mundo_check devuelve 0 con las de ejemplo y 1 con un YAML roto.

Parte B, con terreno_prueba.py (cajas y rampas alineadas con los ejes, perfil exacto y GEO_PRUEBA fija):
- Geometría:
  - β = 5,563°, igual al belt_angle;
  - soporte hacia abajo: con q = 0, la polea activa (0,30, −0,0764); con −10°, la punta; con −β, empate a 1e-9;
  - con q = 30°, un canto sobre el tramo inferior da un ataque de 35,56°;
  - con el xacro real (importorskip de xacro): chasis 0,60/0,40/0,10, ancho 0,08 y masas 20 y 3.
- Reposo:
  - llano: z = 0,0764, sin inclinación y apoyando las cuatro; con −45°, z = 0,29606; con −90°, z = 0,4015;
  - paridad con chasis_core.pose_chasis a ±0,05 mm y ±0,01° en las poses {FR, FL, RR, RL}: (0,0,0,0), (45,45,45,45), (−45,−45,−45,−45), (−90,−90,−90,−90), (−30,−30,0,0), (0,0,−30,−30), (45,45,−45,−45), (−30,−30,10,10) y (−20,−20,−60,−60). Las cinco últimas tienen cabeceos −8,056 / +8,056 / +14,345 / −9,016 / +9,575, en ese orden;
  - una sola oruga bajada (FR a −30°): apoyan FR, RR y RL. Es el cambio documentado frente a chasis_core;
  - rampa de 15° con los brazos a −β: cabeceo −15,00 ± 0,05°;
  - caja de 0,05 bajo las dos orugas izquierdas: balanceo +5,95 ± 0,3°;
  - barra de 0,10 x 0,12 bajo la panza con los brazos a +30°: sin_traccion y z = 0,17. Con los brazos a −30°: de pie sobre las cuatro puntas, z = 0,2215 y con tracción;
  - propiedad, con 600 casos al azar (cajas y rampas de hasta 0,3 m, q entre −90° y 90°): ninguna línea penetra más de 1e-6 y, si no vuelca, el CdG cae dentro del apoyo (±1 mm);
  - colocar no mueve (x, y, yaw).
- Avance:
  - llano igual que la odometría: 1000 pasos con v = 0,053 y w = 0,2 dan lo mismo que odometry_core.integrar a 1e-9;
  - primer contacto, con q en {0, 10, 20, 30, 45, 60, 69}:
    - a h_max − 5 mm sube: el morro gana al menos 1 cm en 2 s;
    - a h_max + 5 mm se para a la distancia de §5.6 (±1 mm), con bloqueado = 'FR'/'FL' y la z sin cambiar;
  - q = 72°: sube 0,050 y no 0,062;
  - pared de 1 m con q entre −20° y 90°: no pasa; tras 10 s empujando, x quieta (±1 mm) mientras integrar() avanza 0,53 m;
  - tablas del prototipo:
    - escalón de 0,20: sube con 30, 40 y 50°; con 20° se para en x = 0,320 (con la cara en 1,0); con 60° se atasca con cabeceo 9,6 ± 1°;
    - escalón de 0,35: con 60° fijos no sube; con el guion 60° -> 20° al pasar de 8° de cabeceo, sí, y acaba con z = 0,4264;
    - bajadas de 0,10 / 0,20 / 0,30 con los brazos a 0: llegan a z = 0,0764 con cabeceo mínimo de −9,5 / −18,3 / −27,6 (±1,5°) y velocidad de cabeceo ≤ 90°/s·1,01;
    - escalera de 0,20/0,25, con las delanteras a 45° y cambio (delanteras, traseras) a (−6, −6) o a (−15, −15) al pasar de 15° de cabeceo: sube. Con (−6, 0) se atasca;
  - rampa lisa: la de 30° sube y la de 45° no (regla de tracción);
  - girar en el sitio con el costado a 5 mm de una pared de 0,5 m: |Δyaw| < 1° y sin penetrar;
  - brazo contra una pared: parado tocando con q = 80°, bajarlo a 20° hace retroceder la base 0,36·(cos 20° − cos 80°) = 0,2758 ± 2 mm;
  - regresiones de §5.8, y colocar un robot mal apoyado no lo desplaza más de 5 mm;
  - determinismo: un paso(dt) y dos paso(dt/2) dan lo mismo a ±1 mm y ±0,1°.
- medir_contacto.py (script, no test): escena de la escalera, 3000 ticks; da la media y el p99.

Parte C, test_marcadores.py:
- estado_json con un Estado falso (SimpleNamespace): prioridades, textos, y el consejo con L1 cuando bloquea FR.
- indicadores: 'choque' solo si está bloqueado; 'estado' solo si no está libre; el color del polígono según el margen; el contrato de claves.
- La conversión de dict a Marker, con pytest.importorskip('visualization_msgs').

Parte D:
- test_mundo_modelo.py:
  - un CUBE u otro tipo raro va a omitidos;
  - TRIANGLE_LIST con colores por vértice;
  - DELETEALL vacía; DELETE borra solo su ns/id;
  - se respeta el orden dentro del array;
  - un frame distinto va a omitidos;
  - el redondeo.
- test_servidor.py: GET /mundo sin datos da 200 con versión 0; con datos, el JSON; siempre no-store.
- test_vista3d.py:
  - sigue habiendo un solo EventSource;
  - aparecen fetch('/mundo' y mundo_v;
  - innerHTML sigue siendo solo el de #orugas;
  - no se repiten ids;
  - ninguna URL externa nueva;
  - textContent en las etiquetas y en el chip.

Parte E, test_mundo_integracion.py (puro, con los mundo_core y contacto_core de verdad):
- llano: igual que chasis_core en las ocho poses por pares.
- escalon:
  - brazos a 0 y v = 0,053 durante 10 s a 50 Hz: x = 0,2985 ± 1 mm y bloqueado;
  - subiendo las delanteras a +40° a 4,69°/s sin soltar el avance: sube y acaba con z = 0,2764 ± 1 mm y cabeceo 0 ± 0,1°.
- practica_rrl:
  - colocar en la salida da z = 0,0764;
  - contra el bordillo del carril 1, con los brazos a 0, se para en x = 0,4985 ± 1 mm;
  - en la salida, girar con w = 0,2 durante 5 s con los brazos en llano no gira (|Δyaw| < 1°: choca con los tabiques); con los brazos a +90, sí gira.

colcon test tiene que seguir con 0 tests saltados: con ROS cargado, los importorskip corren.

======================================================================
§12 VERIFICACIÓN EN VIVO
======================================================================
En el WSL Ubuntu 22.04 con Humble. La hace la Parte E, después de mergear A a D.

0. colcon build y colcon test: pasan todos, 0 saltados. medir_contacto.py: media < 5 ms y p99 < 15 ms.
1. Sin mundo no cambia nada. Se repite la prueba de la tarea 6:
   - lanzar con sim:=true gui:=true rviz:=true teleop:=false y 8 s de /cmd_vel en curva;
   - las dos vistas marcan x = 0,63, y = 0,97 y 117°;
   - ros2 node list no muestra starcrawler_mundo.
2. Llano, con mundo:=llano:
   - /mundo/verdad frente a /odom: menos de 5 mm y 0,2° después de la curva;
   - ros2 topic info /joint_states -v: los publicadores son starcrawler_sim y starcrawler_mundo, sin starcrawler_chasis;
   - el log de la odometría dice "publish_tf desactivado";
   - ros2 run tf2_ros tf2_echo odom base_footprint responde.
3. Escalón con el mando virtual por UDP (el guion de scratchpad\mando_virtual.py, con joy_udp:=true teleop:=true, mandando al puerto 8890):
   - avanzar con los brazos a 0: BLOQUEADO en las dos vistas (chip, flecha y texto en RViz), con x ≈ 0,30 m, y la flecha gris de /odom adelantándose;
   - mantener L1 unos 8,5 s (+40° a 4,69°/s) y avanzar: sube;
   - en la plataforma, z ≈ 0,276 y cabeceo 0;
   - al final baja, con un cabeceo mínimo de unos −18°.
4. Escalera y practica_rrl, con el DS4 real (tools/joy_bridge en Windows, joy_udp:=true) y con el mando web (gui_mando:=true):
   - una vuelta completa;
   - comparar x, y, altura y cabeceo en las dos vistas;
   - mirar el polígono y el CdG en la escalera;
   - recargar el YAML guardándolo (mundo:=ruta absoluta) con RViz abierto.
5. ESP32 en HW_SIMULADO:
   - firmware.sh compilar --simulado, flashear y usbipd attach;
   - lanzar con port:=/dev/ttyUSB0 mundo:=escalon rviz:=true gui:=true joy_udp:=true;
   - repetir el punto 3 con el mismo guion: la parada tiene que salir a ±5 mm de la de sim:=true, y el resto igual, porque la planta del ESP32 imita a sim_core.
6. Modo espejo, con simulate:=true mundo:=escalon (el driver serie no pone el bit 7):
   - ERROR en el log;
   - sin obstáculos;
   - el robot se dibuja y se mueve en llano;
   - chip gris.
7. Vistas:
   - relanzar RViz con el mundo en marcha: se ven los obstáculos (transient local);
   - la página /3d a 375 px con gui_mando:=true: el chip no tapa los controles.

Al terminar, la Parte E propone, sin commitear, la fila de la tabla de la sección 2 de README-CLAUDE y un docs/mundo.md con el uso. Decide Mario.

======================================================================
§13 FUERA DE ALCANCE
======================================================================
- Gazebo y cualquier física dinámica: inercia, rebotes, saltos, patinazo lateral y tracción de un solo lado (un bloqueo de un lado para el robot entero en vez de hacerlo girar). Tampoco el rozamiento según el material: arena, grava, suelo mojado, discos deslizantes o tubos que ruedan.
- Realimentación al robot: el terreno no frena los RMD, no hace perder pasos a los steppers ni dispara límites de corriente. El operador puede aprender maniobras que en el robot real calarían un motor.
- Geometría:
  - voladizos: confined space, crouch under landing y travesaños;
  - tubos y cilindros, zanjas y huecos;
  - elementos apoyados sobre otros ('encima');
  - campos girados y techos no planos;
  - puertas y objetos móviles.
- IMU simulada, EKF, marco map, localización y el paso a map->odom.
- Mundo con el robot real o con el driver serie, salvo el modo espejo.
- Cronómetro, recorrido, puntuación RRL, víctimas y objetivos.
- Editar, reiniciar o recolocar desde la web: se hace desde RViz y con el servicio.
- Cambios en el firmware, sim_core, chasis_core, la odometría y los mensajes.
- Validar los umbrales contra el robot real: nada del robot está verificado en hardware.

======================================================================
§14 GAZEBO
======================================================================
Se mantiene descartado.

Matiz de hechos: gz-sim 6 (Fortress, el de Humble) trae TrackedVehicle y TrackController, basados en el movimiento de la superficie de contacto (Pecka et al.). Tiene problemas conocidos (gz-sim #1662), pero "no modela orugas" ya no es exacto.

Las razones de fondo para no usarlo son otras:
- Con HW_SIMULADO la planta vive en el ESP32. Gazebo sería una segunda planta y rompería la equivalencia entre sim:=true y el ESP32.
- A 5 cm/s y 4,7°/s el régimen es cuasiestático.
- RViz y la web ya cubren las vistas; Gazebo no se vería en la web.
- En WSLg el render es por software.
- Habría que rehacer el modelo en SDF y ajustar fricciones y contactos.

Reabrirlo solo si el modelo cuasiestático se queda corto: vuelcos, caídas por la escalera o inercia. La interfaz lo permite, porque un mundo en Gazebo publicaría la misma TF, las mismas juntas y el mismo /mundo/estado.

======================================================================
§15 ORDEN DE TRABAJO
======================================================================
- A, B y D, en paralelo.
- C, también en paralelo, contra las APIs de §4.3 y §5.9 y los contratos de §6; sus tests no necesitan ni A ni B.
- E, al final.
- Un commit por parte y por causa, sin línea de coautor.
- Los .md de documentación los propone E y los commitea Mario si quiere.