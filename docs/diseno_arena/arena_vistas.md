DISEÑO DEL FRENTE "CÓMO SE VE": el mundo y los indicadores en RViz y en la web /3d (sin tocar el repo)

0. RESUMEN
- El mundo viaja por UN solo canal visual: un visualization_msgs/MarkerArray latcheado en /mundo/marcadores. RViz lo dibuja tal cual y la web lo recibe traducido en la ruta GET /mundo, igual que /modelo traduce /robot_description. Así la web dibuja lo mismo que RViz porque sale del mismo sitio, que es el principio que ya sigue el robot (urdf_modelo.py).
- Los indicadores 3D (contactos, polígono de apoyo, centro de gravedad, choque, texto de estado) son otro MarkerArray, /mundo/indicadores, a 10 Hz. Pasan por el mismo traductor y llegan a la web dentro de /events. El texto del panel sale de un estado compacto, /mundo/estado (JSON).
- En la web no entra ninguna dependencia nueva: todo es three.js 0.160 del núcleo (BoxGeometry, BufferGeometry, InstancedMesh, EdgesGeometry, ArrowHelper, LineDashedMaterial y sombras). Las etiquetas son HTML superpuesto. En el paquete ROS de la GUI solo se añade <depend>visualization_msgs</depend>, que ya viene con el escritorio de ROS.
- Sin mundo todo sigue como hoy: /mundo devuelve la versión 0 sin piezas, se usa plano.rviz y la pose sale de /odom.

1. LO QUE LA VISTA NECESITA DE LOS OTROS FRENTES (contrato que propongo)
a) Marco del mundo, lo más importante. Recomiendo que el mundo viva en 'odom'. En modo mundo, el nodo de terreno publica la TF odom->base_footprint con la pose VERDADERA (plana: x, y, rumbo y z=0), y odometry_node va con publish_tf:=false, un parámetro que ya existe, y sigue publicando /odom. Ventajas para la vista:
   - plano.rviz vale casi tal cual.
   - La flecha Odometry de RViz enseña gratis lo que cree la odometría: cuando el robot se bloquea contra un escalón, las flechas se adelantan al robot.
   - En la web el fantasma de odometría es directamente s.pose.
   Condición: la zona de salida del mundo está en el origen de odom, porque la odometría arranca en (0,0,0).
   La alternativa, un marco 'mundo' con mundo->odom dinámico, que es el patrón map->odom, también funciona. Solo cambia el marco fijo de mundo.rviz, el frame_id de los marcadores y el parámetro marco_fijo de gui_node. La contrapartida es que en RViz la flecha de /odom deja de mostrar la deriva.
b) Alturas, SOLO en las juntas del chasis. chassis_lift_joint pasa a ser la altura absoluta sobre z=0 menos pivot_height, y cabeceo y balanceo son absolutos, los tres en un único JointState. base_footprint se queda siempre en z=0. Motivo visual: si la z fuera en la TF y el cabeceo en las juntas, al pasar el canto de un escalón llegarían en mensajes distintos y el chasis saltaría 20 cm durante un fotograma, en las dos vistas. robot_state_publisher y moverJunta no recortan por límites, pero el límite [0, 0,40] del URDF queda falso (ver decisiones).
c) Un único publicador de chassis_*_joint. Si chasis_node (suelo llano) y el nodo de terreno publican a la vez, el chasis parpadea entre dos alturas en RViz y en la web, porque gui_node mezcla /joint_states en un diccionario.
d) /mundo/verdad, nav_msgs/Odometry en el marco del mundo: la pose verdadera para la web, que hoy lee /odom.
e) /mundo/estado, std_msgs/String JSON a 10 Hz. Esquema v1:
   {"v":1, "estado":"libre|bloqueado|encallado|patina|vuelco|incoherente", "motivo":"...", "consejo":"...", "apoya":[b,b,b,b], "holgura":[m,m,m,m], "panza":b, "margen":m, "cabeceo":rad, "balanceo":rad, "altura":m, "z_suelo":m, "avance_orugas":m/s, "avance_real":m/s, "pieza":"ns/id"|null}
   No lo metería en starcrawler_msgs: firmware.sh copia ese paquete entero al build del ESP32 en cuanto difiere (líneas 39-42), y un mensaje solo de simulación acabaría compilado en el firmware.
f) /mundo/rastro, nav_msgs/Path de la trayectoria verdadera CON z, a 2 Hz y con un punto cada 2 cm. Hace falta porque /odom tiene z=0 y en RViz sus flechas quedan escondidas debajo de rampas y escaleras.

2. RVIZ
2.1 Configuración nueva, starcrawler_description/rviz/mundo.rviz. Es una copia de plano.rviz y el launch la elige cuando mundo no está vacío. plano.rviz se queda limpio para el robot real, sin pantallas en aviso.
- MarkerArray "Mundo": /mundo/marcadores, Depth 1, Durability Policy Transient Local, Reliability Reliable.
- MarkerArray "Apoyo": /mundo/indicadores, Depth 5, Volatile, Reliable.
- Path "Rastro real": /mundo/rastro, color 57;135;229 (el --s1 del rastro de hoy), Billboards de 0,01.
- Odometry en gris 137;135;129 con Keep 300. Se lee como "lo que dice la odometría".
- Vista Orbit con Target Frame base_link, no base_footprint. Sigue la posición del chasis, también en altura: con base_footprint en z=0, arriba de la escalera el robot quedaría fuera del punto focal. Distance 3, Pitch 0,45.
- Vistas guardadas: "Perfil" (rviz_default_plugins/ThirdPersonFollower sobre base_footprint, yaw +90°, pitch 0,1, distancia 2,2), que es LA vista para subir escalones; "Detrás", igual con yaw 180°; y "Cenital" (TopDownOrtho).
- La rejilla sigue con celdas de 0,5 m, como la web.

2.2 Marcadores del mundo, el contrato que debe cumplir quien los genera:
- Un solo mensaje, publicado al arrancar y cada vez que cambie el mundo. El primer marcador es un DELETEALL (action 3) y detrás van las piezas. frame_id = marco fijo, lifetime 0, ids estables (el índice en el fichero del mundo) para que estado.pieza pueda nombrar "escalones/3".
- Espacios de nombres por familia: suelo, rampas, escalones, escaleras, vallas (palés y K-rails), stepfield, paredes, aristas, etiquetas y salida.
- Piezas rectangulares (escalones, plataformas, postes del stepfield, vallas): CUBE.
- Cuñas y rampas: TRIANGLE_LIST. En Humble, rviz2 calcula la normal de cada triángulo como (p1-p0)x(p2-p0), no descarta caras (CULL_NONE) y admite colores por vértice (visto en triangle_list_marker.cpp). Por tanto el orden de los vértices DEBE ir en sentido antihorario visto desde fuera, o la cara sale oscura sin dar ningún error. Lo cubre un test.
- Aristas: un LINE_LIST por pieza con las 12 aristas, de 0,004 m, en #1c1b19 con alfa 0,7. La luz de RViz va pegada a la cámara y la contrahuella vista de frente sale tan clara como la huella: sin aristas, los escalones no se distinguen.
- Etiquetas: TEXT_VIEW_FACING de 0,08 m de alto, 0,4 m por encima de cada elemento, en #c3c2b7: "Escalera 40° · 6 × 18 cm", "Rampas cruzadas 15°", "Stepfield", "Valla de palé 20 cm", "SALIDA". Se apagan desde la casilla del espacio de nombres.
- Zona de salida: LINE_STRIP del cuadrado de 1,2 × 1,2 m sobre el suelo, en blanco con alfa 0,5.
2.3 Paleta del terreno: codificada por altura y neutra, en lugar de madera realista.
- Color = interpolación lineal entre #6b675f (z=0) y #cdc6b6 (z >= 1,0 m), según la z superior de la pieza. En las rampas se aplica por vértice, así que la rampa sale en degradado de abajo arriba.
- Motivo: cada escalón queda un tono más claro que el anterior, se lee en vista cenital, y los colores de las orugas (FR #3987e5, FL #008300, RR #d55181, RL #c98500) siguen destacando. Un tono madera (tostado, cerca de 40°) se comería el ámbar de RL.
- Paredes de carril, que no se pisan: #7a828c con alfa 0,28, para que el robot se vea a través de ellas.
- Todo lo transitable, opaco. No se pone losa de suelo en z=0, que pelearía en profundidad con la rejilla; si hace falta, su cara superior va a -0,002.

3. WEB /3d
3.1 Servidor (servidor.py): GET /mundo, con la misma forma que /modelo. Devuelve MUNDO['json'] o, si no ha llegado nada, {"version":0,"piezas":[],"omitidos":0}, siempre con Cache-Control no-store. Nunca responde 503: la ausencia de mundo es un estado normal.
3.2 gui_node.py, cuatro suscripciones:
- /mundo/marcadores, con QoS reliable + TRANSIENT_LOCAL y depth 1, como robot_description. Se traduce con el módulo PURO nuevo mundo_modelo.py, hermano de urdf_modelo.py: acepta objetos con atributos (el mensaje ROS, o SimpleNamespace en los tests) y aplica ADD, DELETE y DELETEALL sobre un diccionario indexado por (ns, id). Guarda MUNDO['json'] y MUNDO['version'] += 1, y escribe ESTADO['mundo_v'].
- /mundo/indicadores: ESTADO['marcas'] = lista traducida.
- /mundo/estado: ESTADO['terreno'] = JSON validado (tope de tamaño y claves conocidas; lo que viene de fuera no tumba el nodo).
- /mundo/verdad: ESTADO['verdad'] = [x, y, yaw, v, w], como cb_odom.
- Un temporizador a 5 Hz pone a None terreno, marcas y verdad cuando pasan de 0,5 s. Así, si el nodo de terreno muere, los indicadores desaparecen en vez de quedarse congelados diciendo LIBRE.
- Nada de esto pasa por registrar(), para que no cuente en paquetes/s ni en el enlace, igual que hoy pose y joints.
- Parámetro marco_fijo, por defecto 'odom'.
- La geometría del mundo NUNCA va en /events; por /events solo viajan mundo_v, verdad, terreno y marcas (unos 2 KB por evento).
3.3 Formato de /mundo, lo que produce mundo_modelo.py:
{"version":n, "piezas":[{"clave":"escalones/3", "tipo":"caja|triangulos|lineas|tira|esferas|cubos|cilindro|esfera|flecha|texto", "marco":"odom", "pos":[x,y,z], "quat":[x,y,z,w], "escala":[sx,sy,sz], "color":[r,g,b,a], "puntos":[...]?, "colores":[...]?, "texto":"..."?}], "omitidos":k}
Los tipos no soportados (MESH_RESOURCE, POINTS...) se cuentan en omitidos y se avisan por console.warn, como mallas_omitidas.
3.4 En vista3d.py (dentro del script type="module"; el mando no se toca):
- construir() devuelve también links, para poder colgar un marcador del eslabón que diga su frame_id: base_link, crawler_fr_link... Los marcadores en marco_fijo van a la escena; los de otro marco, a omitidos.
- Una función construirMarca(p) común para el mundo y los indicadores. La cuaternión ROS pasa tal cual a THREE.Quaternion(x,y,z,w), porque la escena ya usa Z arriba.
  - caja: BoxGeometry más EdgesGeometry, como el robot.
  - triangulos: BufferGeometry no indexada con computeVertexNormals() (sale sombreado plano), DoubleSide, vertexColors si trae colores, y la escala aplicada al objeto, como hace RViz.
  - lineas y tira: LineSegments y Line (en WebGL siempre de 1 px).
  - esferas y cubos: InstancedMesh.
  - flecha: ArrowHelper.
  - texto: etiqueta HTML.
  - Materiales: MeshStandardMaterial con roughness 0,9 y metalness 0, color en SRGBColorSpace como color(). Si alfa < 1, transparent con depthWrite:false y renderOrder 1.
- Carga reactiva: en cada evento, si s.mundo_v es distinto de la versión cargada, se hace fetch('/mundo'), se desechan (dispose) el grupo y las etiquetas anteriores y se reconstruye. Cubre que el mundo llegue después de abrir la página, que se cambie de mundo y que la GUI se reinicie.
- Etiquetas: una capa #etiquetas dentro de #escena, con pointer-events:none y z-index 1, por debajo del #mando (que está en 2). En cada fotograma se proyecta un Object3D ancla con Vector3.project(camara). Se ocultan detrás de la cámara o a más de 8 m. Estilo: 11px, --tinta2, fondo rgba(13,13,13,.6) y radio 4px. SIEMPRE createElement + textContent, nunca innerHTML: el texto viene de ROS, y además test_el_mando_no_se_regenera ya cuenta los innerHTML.
- Pose del robot: objetivo = s.verdad || s.pose. Se mantienen el suavizado y el reinicio con saltos de más de 1 m. robot.raiz queda en z=0 y la altura va en las juntas.
- Rastros: el real, en --s1 como hoy, con z = terreno.z_suelo + 0,004, porque a z=0 quedaría bajo la escalera. El fantasma de odometría (s.pose, solo si hay verdad) va con LineDashedMaterial en --apagado, más una flecha hueca en la pose de odometría. "Borrar rastro" borra los dos.
- Cámara:
  - En modo seguir, orbita.centro.z = z de base_link en el mundo (getWorldPosition después de mover las juntas), suavizada con tau de 0,3 s para que no pegue saltos en los cantos. Hoy está fija en 0,1.
  - Botones nuevos: "Perfil" (az = yaw + π/2, el = 0,12, sigue el rumbo) y "Detrás" (az = yaw + π, el = 0,35), para conducir con el DS4 en tercera persona.
  - Los botones "Sombras" y "Etiquetas" solo aparecen si hay mundo.
- Sombras, activadas por defecto: renderer.shadowMap (PCFSoft, mapa de 2048 en escritorio y 1024 si el ancho es menor de 760 px). El sol sigue al robot: posición = robot + (3,-2,6) y target = robot, con una cámara de sombra ortográfica de ±4 m. El mundo proyecta y recibe; el robot proyecta. Es la mejor pista de altura en pantalla plana: se ve si la oruga toca el escalón o flota. RViz no tiene sombras; allí suplen las aristas, los contactos y la plomada.
- Rendimiento: una malla por marcador vale para una pista tipo RoboCup (unas 100 piezas). Si se mide que baja de 30 fps en el móvil, la geometría opaca se fusiona en una BufferGeometry con colores por vértice, a mano (unas 20 líneas), porque mergeGeometries está en addons, no en el núcleo.

4. INDICADORES: bloqueado, apoyado y demás
Principio: neutro cuando todo va bien y color solo cuando importa. Rojo solo para vuelco. EMERGENCIA y SIN ENLACE siguen siendo los avisos de más rango.
- Contacto de las orugas: en /mundo/indicadores, SPHERE_LIST "apoyo/contactos" de 3,5 cm en blanco #ffffff, en los puntos de contacto (marco del mundo). En la web, además, una columna nueva en "Orugas": "● apoya" en --tinta, o "○ 3,4 cm" en --apagado, sacado de apoya[] y holgura[]. No se recolorean las orugas: el verde de "apoya" chocaría con FL. Queda libre el gris de marcarEncoder, que sigue significando encoder caído.
- Panza apoyada (ENCALLADO, el fallo típico en el canto de un escalón): SPHERE_LIST "apoyo/panza" en #fab219, y en la web las aristas del chasis pasan a ámbar con opacidad 0,9.
- Polígono de apoyo y centro de gravedad (sobre todo en escaleras): LINE_STRIP cerrado "apoyo/poligono" de 0,012 m por los contactos, en orden de envolvente sobre xy. SPHERE "apoyo/cdg" de 4,5 cm y LINE_LIST "apoyo/plomada" desde el centro de gravedad hasta la z del contacto más bajo. Color según el margen: #c3c2b7 si es mayor de 0,10 m, #fab219 entre 0,03 y 0,10, y #d03b3b por debajo de 0,03.
- Bloqueado: ARROW "apoyo/choque" desde el punto de la cara que frena, 0,15 m hacia el robot, en #fab219. En la web, además, la pieza estado.pieza con emisivo ámbar (0x5a3c00) y la línea "orugas 0,20 m/s · avance real 0,00 m/s": que las orugas giren sin avanzar es el síntoma de bloqueo.
- Texto de estado: TEXT_VIEW_FACING "apoyo/estado" en frame base_link, frame_locked true y sello 0 (la última TF), a 0,30 m sobre el chasis: BLOQUEADO, ENCALLADO o PATINA en #fab219, y VUELCO en #d03b3b. Si el estado es libre, no se pinta.
- INCOHERENTE, un aviso de depuración: con el ESP32 en HW_SIMULADO la planta no sabe del terreno y un brazo puede atravesar una pieza. La web pone la oruga afectada con aristas rojas, y el chip dice "SIM: FR atraviesa escalones/3 (el ESP32 no sabe del terreno)", para que no se tome por un fallo del robot.
- Chip #terreno en la escena: arriba al centro, con z-index 1, por debajo del mando. Fondo del color del estado, texto #0d0d0d en negrita y con espaciado, como #sinEnlace. Contiene estado · motivo · consejo, por ejemplo "BLOQUEADO · el morro choca con un escalón de 20 cm · sube las delanteras (L1)". Los consejos usan las etiquetas del mando web y del DS4: L1/L2 las delanteras, R1/R2 las traseras. En #escena.estrecho va abajo al centro, por encima de los controles; hay que probarlo en un móvil con el mando activo. Si hay mundo pero el estado lleva más de 0,5 s sin llegar: "terreno: sin datos", en gris.
- Sección "Terreno" en el aside, oculta sin mundo: estado, cabeceo y balanceo en grados (ámbar por encima de 25° y rojo por encima de 35°), altura del chasis, margen de estabilidad, avance real frente al de las orugas, y error de odometría (distancia entre verdad y pose). "Posición (odom)" pasa a "Posición (real)" cuando hay verdad.
- Lifetime de 0,5 s en todos los indicadores, más un DELETEALL al principio de cada publicación. En la web, marcas se sustituye entera en cada evento. Los marcadores con frame_locked cuelgan del grupo de su eslabón y siguen al modelo suavizado; los que van en el marco del mundo pueden adelantarse ~1,6 cm a 0,2 m/s por el suavizado de 80 ms, que no se aprecia.

5. FICHEROS QUE CAMBIARÍAN (no he tocado ninguno)
- starcrawler_gui:
  - mundo_modelo.py (nuevo, puro).
  - gui_node.py (cuatro suscripciones, temporizador de caducidad, parámetro marco_fijo).
  - servidor.py (GET /mundo).
  - vista3d.py (renderizador de marcas, etiquetas, sombras, cámara, chip y panel).
  - package.xml (visualization_msgs).
  - Tests: test_mundo_modelo.py nuevo, más ampliaciones de test_servidor.py y test_vista3d.py.
- starcrawler_description: rviz/mundo.rviz (nuevo). Posiblemente también los límites de chassis_lift_joint del xacro (decide Mario).
- starcrawler_bringup/robot.launch.py: elegir mundo.rviz y publish_tf:=false de la odometría cuando haya mundo. Esto es del frente de lanzamiento; aquí solo lo dejo apuntado.
- Del frente del mundo y la física: la función pura que genera los marcadores y el estado, siguiendo el contrato de la sección 2.2 y el esquema de 1.e.

6. PRUEBAS Y CÓMO DAR ESTO POR VERIFICADO
- pytest, sin ROS:
  - mundo_modelo: CUBE pasa a caja con pos, quat y escala; DELETEALL vacía; DELETE borra solo su (ns, id); los tipos raros suman en omitidos; se respeta el orden de acciones dentro de un mismo array.
  - servidor: /mundo sin datos da versión 0 con 200; con datos, el JSON.
  - vista3d: sigue habiendo un solo EventSource; aparece fetch('/mundo') y mundo_v; innerHTML sigue siendo solo el de #orugas; no se repiten ids; no hay más URL externa que la de three.js.
  - Generador del mundo: cada triángulo tiene la normal hacia fuera (el producto escalar con el vector centro de pieza -> centro de cara es positivo), los colores están en [0,1] y los ids no se repiten dentro de un ns.
- Prueba a ojo, como en la tarea 6:
  1. sim:=true con mundo, RViz (mundo.rviz) y /3d a la vez.
  2. Ir contra el escalón con los brazos abajo: el chip y la flecha salen en las dos vistas, y la flecha gris de /odom se adelanta al robot en RViz.
  3. Subir las delanteras, trepar, bajar, y subir una escalera: vigilar el polígono y el centro de gravedad.
  4. Comparar los números de las dos vistas: x, y, altura del chasis y cabeceo.
  5. Repetirlo con el ESP32 en HW_SIMULADO y el DS4.
  6. ros2 topic info /joint_states -v tiene que dar un solo publicador de chassis_*_joint.
  7. Mirar la página a 375 px de ancho con gui_mando:=true.

7. GAZEBO
Por la parte visual no hace falta reabrirlo. MarkerArray y three.js cubren RViz y la web, y Gazebo no se vería en la web ni modela las orugas.