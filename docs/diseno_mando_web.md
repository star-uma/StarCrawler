# Diseño del mando desde la GUI web (issue #18)

Diseño con el que se implementó el mando web (28-29/09/2026). Salió de una revisión en cuatro frentes (seguridad física, integración con ROS 2, red y operador) y de una síntesis. Está implementado y probado en vivo con el ESP32 (`HW_SIMULADO`) y los muxes: los seis casos de la sección 9. Lo que cambió al implementarlo o en la revisión adversarial del código (sobre todo: plazo de /mando a 180 ms, rechazo de órdenes retrasadas en la red, caducidad de teclas sin autorrepetición y servidor en IPv6 e IPv4) está en el código y en `docs/ros2.md` §6; aquí queda el razonamiento de partida.

Decisiones de Mario sobre las preguntas abiertas: el DS4 tocado gana a la web (30 > 20 > 10); SHARE sigue sin engancharse; el estado seguro sigue sin par; el paso 0 fue en su propio commit; la clave es aleatoria salvo `gui_clave:=`; `gui_mando` sin teleop se permite con aviso.

COMPROBADO EN EL CÓDIGO ANTES DE DECIDIR
- app.c:183 y 203. Hay una sola marca, tickComando, para los dos tópicos, y se toma con xTaskGetTickCount. El watchdog (287-300) la compara con hw_millis(), que es esp_timer (hw.c:38): son dos relojes distintos. Al vencer no toca las consignas. En app.c:191, emergencia es la del último mensaje. TaskWatchdog sí es coherente (ticks contra ticks).
- twist_mux de Humble (topic_handle.hpp:189-197, visto en el WSL). Solo publica en el callback de la fuente que tiene la prioridad; cuando una fuente caduca no emite nada.
- http.server del WSL con Humble: protocol_version es HTTP/1.0, es decir, una conexión y un hilo por POST.
- joy_logic.py:226-249. El preset del DS4 solo se borra con una orden manual o con SHARE. teleop_node.py:106-115 no lo borra cuando caduca /joy.
- sim_core.py:188-203 y 223 copian la marca única. sim_node y driver_node se suscriben con la QoS por defecto (fiable).
- El typesupport de micro-ROS que hay en ~/microros_ws se salta un header.frame_id más largo que su capacidad y acepta el mensaje igualmente. Aun así, frame_id no se usa.
- vista3d.py:143-145. Todo el JS va en el módulo que importa three.js, y panel() reescribe innerHTML a 10 Hz (359-366).

1. CADENA DE SEGURIDAD
a) Página. Funciona como hombre muerto: solo manda mientras hay algo pulsado o un preset enganchado en curso, y ante cualquier duda lo suelta todo.
b) gui_node. Publica a 50 Hz solo mientras la última orden aceptada tenga menos de 0,2 s.
c) Muxes. Las fuentes web y joy_activo caducan a los 0,25 s, por debajo de los 300 ms del watchdog, así que el relevo al DS4 en reposo llega antes de que salte.
d) crawler_mux. Hace el OR de las emergencias de todas las fuentes frescas y engancha la emergencia web hasta que se rearma a propósito.
e) ESP32. Sin /crawler/command fresco pasa a estado seguro. Sin /cmd_vel fresco pone la tracción a 0.
En el peor caso el hombre muerto tarda unos 0,5 s en actuar, que son menos de 2,7 cm a 0,053 m/s.

2. GRAFO
Tabla única de fuentes. Va en las dos secciones de config/mux.yaml y un test comprueba que coinciden:
- joy: cmd_vel_joy + crawler/command_joy | prioridad 10 | timeout 0,5 | no engancha. Es el teleop, que publica siempre (ceros en reposo, que sujetan con par).
- web: cmd_vel_web + crawler/command_web | prioridad 20 | timeout 0,25 | engancha. Es gui_node.
- joy_activo: cmd_vel_joy_activo + crawler/command_joy_activo | prioridad 30 | timeout 0,25 | no engancha. Es el teleop, solo con el DS4 tocado y 0,2 s de cola.
- navigation: cmd_vel_nav (sin orugas) | prioridad 100 | timeout 0,5 | reservado.
Salidas: twist_mux publica /cmd_vel y crawler_mux publica /crawler/command.
Tópicos nuevos:
- crawler_mux/activa (std_msgs/String, transient_local, depth 1, solo al cambiar). Valores: '' | joy | web | joy_activo | emergencia.
- crawler_mux/rearmar (std_msgs/Empty).
QoS por defecto (reliable, 10) en todo lo nuevo. Casa con el ESP32 (best effort) y con sim_node y driver_node.
Cabeceras: stamp = now del nodo que publica; frame_id siempre vacío.

3. PASO 0: FIRMWARE, SIMULADOR Y DRIVER (requisito para usar gui_mando con el robot; commit propio)
Por qué: crawler_mux se mete en el camino de SHARE. Si muere y el ESP32 sigue conduciendo con /cmd_vel, la parada no llega.
- app.c:
  · tickComando se sustituye por msCmdVel y msCrawler (volatile uint32_t). Se toman con hw_millis() en cb_cmd_vel y cb_crawler y se inicializan con hw_millis() en app_main.
  · Los callbacks descartan el mensaje si algún campo no es finito (isfinite), sin refrescar la marca.
  · En TaskControl: si msCmdVel vence, objIzq=objDer=0 y la tracción baja por rampa a 0 manteniendo el par. Si msCrawler vence, estado seguro como hoy (emer || vencido) con CC_ERR_WATCHDOG. /crawler/command hace de latido de la parada.
  · Con la caducidad por tópico no hace falta borrar las consignas al entrar en estado seguro.
- sim_core: _t_traccion y _t_orugas con las mismas reglas. sim_node descarta los valores no finitos.
- driver_node (camino serie): sin /crawler/command fresco manda emergencia=True y tracción 0.
- scripts/instalar_pc_abordo.sh y docs/ros2.md §4: el ejemplo de ros2 topic pub /cmd_vel tiene que publicar también /crawler/command a 20 Hz.
- Tests en test_sim_core:
  · solo cmd_vel → estado seguro;
  · solo orugas → tracción 0 sin estado seguro;
  · cmd_vel vencido y después solo orugas → la tracción no se reanuda;
  · test_con_consignas_frescas_no_salta se adapta para que mande los dos.
- Después, probarlo en el banco con HW_SIMULADO: que compile no quiere decir que funcione.

4. starcrawler_common/orugas.py (nuevo; única fuente de verdad del esquema del mando)
- Se mueven aquí desde joy_logic PAR_DELANTERO, PAR_TRASERO y PRESETS_DEG, e inclinacion(arriba, abajo, izq, der), que hoy es LogicaMando._inclinacion.
- Funciones nuevas:
  · componer_incremento(delanteras, traseras, inclinacion): signo por oruga;
  · traccion(avance, giro, factor, max_lineal, max_angular) → (lineal, angular), con giro positivo a la derecha que da angular.z negativo.
- Valores por defecto: MAX_LINEAL 0,053, MAX_ANGULAR 0,20, FACTOR_LENTO 0,5, S_PRESET 0,5.
- joy_logic los importa de aquí sin cambiar su comportamiento; sus tests no se tocan.
- Nuevo test_orugas.py: signos de la tracción, pares, inclinaciones y saturación de la suma.

5. starcrawler_teleop
- joy_logic.py:
  · Salida.activo se activa con cualquier entrada no neutra: stick fuera de la zona muerta, L1, L2, R1, R2, cruceta, botón de preset pulsado o SHARE. Sigue activo hasta S_COLA_ACTIVO = 0,2 s después de la última.
  · No cuentan L3, OPTIONS ni el preset ya enganchado.
  · Nuevo cancelar_preset(): pone _posicion_vigente=False y _preset_pulsado=-1.
  · Tests: cada entrada, la cola, el preset enganchado no cuenta como activo, y cancelar_preset.
- teleop_node.py:
  · Publica como hoy y, solo si s.activo, los mismos mensajes en cmd_vel_activo y crawler/command_activo.
  · Cuando caduca /joy llama a cancelar_preset() y no publica en los canales activos.
  · Se suscribe a crawler_mux/activa (transient_local) y llama a cancelar_preset() si el valor no es '' ni una de sus fuentes. El parámetro fuentes_propias vale por defecto [joy, joy_activo].
- mux_core.py (puro):
  · Tipos: Fuente(nombre, prioridad, timeout_s, engancha) y Orden(incremento[4], usar_posicion, objetivo_rad[4], emergencia). EMERGENCIA es la orden con incremento 0, usar_posicion false, objetivo 0 y emergencia true.
  · recibir(nombre, orden, t). Guarda la orden con su hora de llegada.
    - Con el enganche puesto, devuelve EMERGENCIA solo si este mensaje acaba de ponerlo; si no, None.
    - Si la orden pide emergencia: engancha si la fuente es de las que enganchan, y devuelve EMERGENCIA al momento, tenga la prioridad que tenga.
    - Si la fuente es la fresca de más prioridad, devuelve su orden, o EMERGENCIA si alguna fuente fresca la pide.
    - En otro caso, None.
  · tick(t): EMERGENCIA mientras dure el enganche.
  · rearmar(t) → bool: solo quita el enganche si ninguna fuente fresca pide emergencia.
  · activa(t): 'emergencia' si hay enganche; si no, la fuente fresca de más prioridad, o ''.
  · La frescura se mide con la hora de llegada según el reloj del nodo, nunca con header.stamp.
- crawler_mux_node.py (entry point crawler_mux):
  · Lee topics.<n>.{topic, timeout, priority, engancha} de mux.yaml y abre una suscripción por fuente.
  · Publica crawler/command.
  · Timer a 20 Hz que llama a tick() y publica activa cuando cambia.
  · Se suscribe a rearmar y deja el resultado en el log.
- config/mux.yaml sustituye a twist_mux.yaml. Lleva una sección twist_mux (joy, web, joy_activo, navigation; use_stamped: false) y otra crawler_mux (joy, web, joy_activo). La etiqueta joystick pasa a llamarse joy.
- Tests:
  · test_mux_core.py:
    - prioridad;
    - caducidad por hora de llegada;
    - la emergencia de una fuente inferior sale al momento;
    - OR de las emergencias;
    - la web engancha y joy no;
    - tick solo con el enganche puesto;
    - que la web caduque no rearma;
    - el rearme se rechaza con una emergencia fresca;
    - los valores de activa;
    - EMERGENCIA sale limpia.
  · test_mux_yaml.py: mismas prioridades y timeouts por nombre en las dos secciones; web y joy_activo por debajo de 0,3 s; joy_activo > web > joy.
- package.xml: depend std_msgs. setup.py: el entry point y mux.yaml en data_files.

6. starcrawler_gui
6.1 mando_web.py (puro)
- validar_mando(bytes):
  · json.loads con parse_constant que rechaza NaN e Infinity, más math.isfinite para casos como 1e309;
  · conjunto exacto de claves;
  · tipos comprobados con type(x) is ...;
  · null solo en inclinar y preset;
  · cualquier fallo, 400 sin publicar nada.
- Cuerpo de POST /mando (num = int o float, nunca bool):
  · sesion: [A-Za-z0-9]{8,32}
  · seq: entero ≥ 0
  · avance: num en [-1, 1]
  · giro: num en [-1, 1], + = derecha
  · lento: bool
  · delanteras, traseras: -1 | 0 | 1
  · inclinar: null | arriba | abajo | izq | der
  · preset: null | 0..3
- comprobar_cabeceras(ruta, cabeceras, mando, clave). Se ejecuta antes de leer el cuerpo:
  · 403 si mando=false;
  · 401 en /mando y /rearmar si falta X-StarCrawler-Token o no coincide (hmac.compare_digest);
  · 415 en /mando si Content-Type no es application/json;
  · 411 o 413 si Content-Length falta, es ≤ 0 en /mando o pasa de 1024.
- Arbitro(max_lineal, max_angular, factor_lento, s_preset, preset_max_s). Constantes: FRESCURA 0,2 s, SUELTA_DUENO 1,0 s, VENTANA_EMERGENCIA 0,2 s, TOLERANCIA_PRESET 2°.
  · mando(orden, ip, t, enlace, activa, mux_ok), comprobando en este orden:
    - 409 si activa es 'emergencia';
    - 503 'sin mux' si no hay mux;
    - 409 'otro puesto' si otra sesión es dueña y habló hace menos de 1 s; si no, esta sesión pasa a dueña, se reinicia seq y el cambio va al log con la IP;
    - 409 si seq no crece;
    - 503 'sin enlace' si la orden no es neutra y no hay enlace;
    - si pasa todo, guarda la orden y responde 200.
  · emergencia(t): borra la orden, el dueño y el preset, y abre la ventana de emergencia.
  · salida(t, robot{elev[4], encoder_ok[4], enlace}):
    - En la ventana de emergencia, Twist a 0 y emergency_stop.
    - Sin una orden de menos de 0,2 s devuelve None y olvida el preset.
    - Con una orden reciente compone la consigna con traccion() y componer_incremento().
  · Preset:
    - El temporizador arranca cuando cambia el valor de preset.
    - Una orden manual de orugas lo termina como 'cancelado'.
    - Tras s_preset seguidos pasa a 'en curso': use_position, objetivo radians(PRESETS_DEG[k]) en las cuatro orugas e incremento 0.
    - Termina como 'llegado' si todas están a 2° o menos; 'sin encoder' si alguna tiene encoder_ok false; 'sin enlace'; o 'tiempo' al pasar preset_max_s.
    - Una vez terminado se ignora ese preset hasta que llegue otro valor.
  · estado(t): el diccionario para /events.
- test_mando_web.py:
  · valores inválidos: NaN, ±Infinity, 1e309, null, 'false', true o un float en delanteras, claves de más o de menos, rangos, sesion mala;
  · cabeceras, sin llegar a leer el cuerpo;
  · dueño, 409 y liberación a 1 s; seq; frescura;
  · emergencia; 409 con el enganche puesto; sin enlace (lo neutro sí pasa);
  · signos: giro a la derecha da angular negativo, delanteras +1 da [1,1,0,0], arriba da [1,1,-1,-1]; lento;
  · el ciclo completo del preset.

6.2 servidor.py (sin rclpy; ManejadorRos sale de gui_node)
- Rutas GET: las de ahora. timeout = 5 en el manejador.
- El Arbitro, un Lock, la clave, mando y las funciones de estado del robot y de rearme le llegan como atributos de clase.
- POST /mando: cabeceras, leer exactamente Content-Length bytes, validar y Arbitro.mando bajo el Lock. Responde JSON {ok, motivo}.
- POST /emergencia: solo pide mando=true y no pide clave, porque cualquiera puede parar. Cuerpo de 1024 bytes o menos, que se ignora. Responde 202.
- POST /rearmar: pide mando y clave. Publica Empty y responde 202; la confirmación llega por /events.
- Sin do_OPTIONS y sin CORS: una web ajena recibe 501 en el preflight.
- test_servidor.py con un servidor en 127.0.0.1:0:
  · OPTIONS → 501;
  · sin clave → 401 sin esperar al cuerpo;
  · text/plain → 415;
  · mando=false → 403.

6.3 gui_node.py
- Parámetros:
  · mando: false por defecto;
  · mando_token: vacío = secrets.token_hex(5) en cada arranque;
  · max_lineal, max_angular, factor_lento, s_preset: por defecto los de orugas.py; el launch pasa los de ds4.yaml;
  · preset_max_s: 40.0;
  · http_port y abrir_navegador, como ahora.
- Con mando activo:
  · Publica Twist en cmd_vel_web, CrawlerCommand en crawler/command_web y Empty en crawler_mux/rearmar. Se suscribe a crawler_mux/activa.
  · Timer a 50 Hz: llama a salida() bajo el Lock y publica los dos mensajes, siempre nuevos. Escribe ESTADO['mando'] con el LOCK.
  · mux_ok = get_subscription_count() > 0 en los dos publicadores.
- cb_estado: registrar(seguridad=bool(msg.safety_active)) y guarda crawler_angle y encoder_ok para el árbitro.
- Log al arrancar:
  · 'Mando web ACTIVO: http://<ip-del-robot>:8000/3d#t=<clave>';
  · aviso si three.js no tiene copia local;
  · aviso si arranca sin teleop, porque entonces no hay parada física.
- ESTADO['mando']: {habilitado, s_preset, activa, dueno {id de 6 caracteres, ip}, preset {k, objetivo_deg, estado}, pedido [lineal, angular], mux_ok}, más ESTADO['seguridad'].

6.4 vista3d.py (/3d)
- Estructura:
  · MANDO_HTML y MANDO_JS van en un <script> clásico antes del módulo 3D.
  · Ese script crea el único window.estadoRobot = new EventSource('/events'); el módulo se cuelga de él con addEventListener.
  · Con mando.habilitado=false se ve una sola línea ('Mando web desactivado (gui_mando:=true)') y no hay atajos de teclado.
- Clave:
  · Se lee del fragmento #t=, se guarda en sessionStorage (con try/catch) y se limpia la URL con history.replaceState(null, '', '/3d').
  · Sin clave solo se ve EMERGENCIA y el aviso 'Mando bloqueado: abre el enlace con clave del log'.
- Controles superpuestos a #escena:
  · EMERGENCIA arriba a la derecha, siempre visible.
  · Pad circular abajo a la izquierda, con un 10 % de zona muerta.
  · Abajo a la derecha: delanteras L1▲/L2▼, traseras R1▲/R2▼, cruceta ▲▼◀▶ para inclinar, y los presets ✕ -45°, ○ 0°, □ +45°, △ +90°, que se mantienen s_preset con un relleno que avanza.
  · Parar, Lento (conmutador, como L3) y Rearmar (solo con enganche y clave, manteniendo 1 s).
  · CSS: touch-action:none, user-select:none, -webkit-touch-callout:none y 48 px como mínimo; contextmenu cancelado; tabindex=-1; nunca se regeneran con innerHTML.
- Punteros:
  · setPointerCapture al pulsar.
  · pointerup, pointercancel y lostpointercapture sueltan ese control.
  · El estado se guarda por pointerId, para poder usar dos dedos a la vez.
- Teclado:
  · Se usa e.code; se ignoran e.repeat y las pulsaciones con ctrl, meta o alt; preventDefault en keydown y en keyup.
  · Asignación: WASD conducir, flechas inclinar, R/F delanteras, T/G traseras, 1-4 presets (mantener), espacio emergencia, Esc Parar.
  · Rearmar nunca por teclado.
- Envío:
  · Un bucle a 20 Hz arma el estado completo y lo manda mientras hay algo pulsado o un preset en curso.
  · Al soltarlo todo manda neutro durante 250 ms y deja de mandar.
  · Una sola petición en vuelo; si hay otra pendiente, solo se guarda la última.
  · AbortController a 300 ms.
  · sesion aleatoria con crypto.getRandomValues y seq creciente.
- Soltar todo: borra todo el estado pulsado, manda neutro y obliga a volver a pulsar; un puntero que siga apoyado no cuenta hasta que se levante. Se dispara con:
  · blur, visibilitychange a hidden, pagehide o freeze;
  · más de 250 ms entre dos ticks;
  · un POST fallido, un 4xx o 5xx, o más de 300 ms sin respuesta ('SIN MANDO');
  · /events sin mensajes durante 1 s ('SIN CONEXIÓN CON EL PC');
  · activa distinto de web, '' o joy mientras se manda;
  · la emergencia.
- Preset:
  · Si se suelta antes de s_preset, se cancela. Si no, queda enganchado y se sigue mandando k.
  · Lo terminan Parar o Esc, una orden manual de orugas, la emergencia, soltar todo, o que /events diga que ha acabado, mostrando el motivo.
  · Mientras dura se ve por brazo, p. ej. 'FR +12° → +90°'.
  · Conducir no lo cancela.
- Emergencia:
  · Primero suelta todo y después manda POST /emergencia cada 100 ms hasta ver activa='emergencia'.
  · Mientras tanto muestra 'PEDIDA…'; cuando se confirma, 'ENGANCHADA'; si a los 2 s no se ha confirmado, 'NO CONFIRMADA: usa SHARE'.
- Tira de estado:
  · Manda: TÚ / otro navegador (IP) / MANDO FÍSICO / nadie / EMERGENCIA WEB.
  · Robot: obedece / en reposo: tracción libre (bit 6) / EMERGENCIA.
  · 'sin mux' y los dos enlaces por separado (navegador-PC y PC-ESP32).
  · Velocidad pedida frente a medida.
- test_vista3d.py:
  · el mando está fuera del módulo y no nombra THREE;
  · hay un solo EventSource;
  · touch-action:none y tabindex=-1 en los controles;
  · usa X-StarCrawler-Token y replaceState;
  · las rutas coinciden con las del servidor.

7. LAUNCH
- Argumentos nuevos: gui_mando (false por defecto) y gui_clave (vacío).
- gui_node arranca con gui o con gui_mando.
  · mando = ParameterValue(gui_mando, bool) y mando_token = gui_clave.
  · Los topes se leen de ds4.yaml con yaml y get_package_share_directory, para que el DS4 y la web usen los mismos.
- twist_mux y crawler_mux arrancan con teleop o gui_mando, con mux.yaml y respawn=True (respawn_delay 1,0).
- Remaps del teleop: cmd_vel → cmd_vel_joy, cmd_vel_activo → cmd_vel_joy_activo, crawler/command → crawler/command_joy y crawler/command_activo → crawler/command_joy_activo.

8. LÍMITES CONOCIDOS (a docs/ros2.md §Mando web y a la #18; en el código, comentarios escuetos)
- Si gui_node se cae, la web deja de mandar, pero una emergencia ya enganchada sigue en crawler_mux.
- Si crawler_mux se reinicia, pierde el enganche. Mientras está caído el ESP32 queda en estado seguro; al volver manda la fuente fresca.
- El estado seguro deja la tracción sin par.
- La clave viaja en claro y aparece en /rosout.

9. ORDEN Y VERIFICACIÓN
Un commit por causa, sin commitear docs por iniciativa propia:
1) paso 0;
2) orugas.py;
3) teleop;
4) crawler_mux y mux.yaml;
5) mando_web, servidor y gui_node;
6) página;
7) launch.

Verificación: colcon test en Humble y, con sim:=true gui_mando:=true joy_udp:=true, estas pruebas:
- cerrar la pestaña mientras se conduce: tiene que parar en 0,5 s o menos;
- tocar el DS4 mientras manda la web: gana el DS4;
- emergencia web y cerrar la pestaña: sigue enganchada;
- rearmar con SHARE pulsado: se rechaza;
- matar crawler_mux: estado seguro;
- preset del DS4, luego la web, luego soltar: el preset no vuelve.
Después, el paso 0 en el banco con HW_SIMULADO.