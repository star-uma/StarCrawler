"""
vista3d.py — el robot moviendose por el plano, en el navegador
==============================================================
Modulo PURO: solo la pagina y dos utilidades de texto. La sirve gui_node
en /3d y lee del mismo /events que la telemetria 2D, con dos campos mas:

    pose    [x, y, yaw, v, w]   de /odom
    joints  {nombre: posicion}  de /joint_states

La geometria no esta escrita aqui: la pagina pide /modelo, que es el URDF
de /robot_description ya traducido por urdf_modelo.py. Dibuja lo mismo
que RViz, con los mismos signos, porque sale del mismo sitio.

Con un mundo simulado (mundo_node) llegan ademas:

    mundo_v  version de la geometria; al cambiar, la pagina pide /mundo
    verdad   [x, y, yaw, v, w, z] de /mundo/verdad (z de base_link)
    terreno  el JSON de /mundo/estado, validado
    marcas   /mundo/indicadores ya traducidos (mundo_modelo.py)

Con verdad, el robot va donde esta de verdad y /odom queda como rastro
fantasma. La altura, el cabeceo y el balanceo llegan en las juntas.

three.js se carga de una copia local si existe (static/, ver README del
paquete) y si no del CDN. Sin internet y sin copia, la pagina lo avisa.

La pagina lleva tambien el mando web (issue #18, docs/ros2.md §Mando web),
que solo aparece si /events trae mando.habilitado. MANDO_HTML va encima de
la escena y MANDO_JS en un <script> clasico antes del modulo: se puede
mandar y parar aunque three.js no cargue. MANDO_JS abre el unico
EventSource de la pagina (window.estadoRobot) y el modulo se cuelga de el.
"""
import json

from starcrawler_common.orugas import PRESETS_DEG, S_PRESET

THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js'

# Botones de preset del DS4 en el orden de PRESETS_DEG, y su sitio en el
# rombo: X abajo, O derecha, cuadrado izquierda, triangulo arriba
SIMBOLOS_PRESET = ('✕', '○', '□', '△')
_SITIO_PRESET = ('b', 'd', 'i', 'a')

# Donde se inserta el enlace a esta vista en la pagina 2D de dashboard.py
_ANCLA_2D = '<span class="sub" id="info">esperando datos…</span>'
_ENLACE_2D = ('<a href="/3d" style="margin-left:auto;color:var(--s1);'
              'text-decoration:none">Vista 3D →</a>')


def pagina_3d(url_three: str) -> str:
    return PAGINA_3D.replace('__THREE_URL__', url_three)


def enlazar_desde_2d(pagina: str) -> str:
    """Anade el enlace a /3d en la cabecera de la pagina 2D."""
    return pagina.replace(_ANCLA_2D, _ANCLA_2D + _ENLACE_2D, 1)


def _grados(d: float) -> str:
    return '%+.0f°' % d if round(d) else '0°'


def _boton_preset(k: int) -> str:
    return ('<button class="control preset %s" data-c="p%d" tabindex="-1" '
            'title="mantener · tecla %d"><span class="relleno"></span>'
            '<b>%s</b><small>%s</small></button>'
            % (_SITIO_PRESET[k], k, k + 1, SIMBOLOS_PRESET[k],
               _grados(PRESETS_DEG[k])))


_PAGINA = r"""<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<title>StarCrawler — vista 3D</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  /* Misma paleta que la telemetria 2D */
  :root {
    --pagina:#0d0d0d; --superficie:#1a1a19; --borde:rgba(255,255,255,.10);
    --tinta:#ffffff; --tinta2:#c3c2b7; --apagado:#898781;
    --rejilla:#2c2c2a; --eje:#383835;
    --s1:#3987e5; --s2:#008300; --s3:#d55181; --s4:#c98500; /* FR FL RR RL */
    --ok:#0ca30c; --critico:#d03b3b; --aviso:#fab219;
  }
  * { box-sizing:border-box; margin:0; }
  html, body { height:100%; }
  body { background:var(--pagina); color:var(--tinta); display:flex;
         flex-direction:column; gap:12px; padding:14px;
         font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }
  .tarjeta { background:var(--superficie); border:1px solid var(--borde);
             border-radius:10px; padding:12px 14px; }
  .cab { display:flex; align-items:center; gap:14px; flex-wrap:wrap; }
  h1 { font-size:16px; letter-spacing:.06em; }
  h2 { font-size:11px; color:var(--apagado); text-transform:uppercase;
       letter-spacing:.1em; margin:14px 0 6px; font-weight:600; }
  h2:first-child { margin-top:0; }
  a { color:var(--s1); text-decoration:none; }
  .punto { width:10px; height:10px; border-radius:50%; background:var(--critico); }
  .punto.on { background:var(--ok); }
  .sub { color:var(--tinta2); font-size:12px; }
  main { flex:1; display:flex; gap:12px; min-height:0; }
  #escena { flex:1; position:relative; overflow:hidden; min-height:320px;
            border-radius:10px; border:1px solid var(--borde); }
  #escena canvas { display:block; width:100%; height:100%; touch-action:none; }
  #aviso { position:absolute; inset:0; display:flex; align-items:center;
           justify-content:center; text-align:center; padding:24px;
           color:var(--tinta2); background:rgba(13,13,13,.72); }
  #aviso[hidden] { display:none; }
  #sinEnlace { position:absolute; top:10px; left:10px; padding:4px 10px;
               border-radius:6px; background:var(--critico); font-size:12px;
               font-weight:600; letter-spacing:.06em; }
  #sinEnlace[hidden] { display:none; }
  aside { width:250px; overflow:auto; }
  .dato { display:flex; justify-content:space-between; font-size:13px;
          color:var(--tinta2); padding:1px 0; }
  .dato b { color:var(--tinta); font-weight:600; }
  .num { font-variant-numeric:tabular-nums; }
  .oruga { display:grid; grid-template-columns:12px 28px 1fr auto auto;
           gap:8px; align-items:center; font-size:13px; padding:2px 0; }
  .oruga i { width:12px; height:12px; border-radius:3px; }
  .oruga .mal { color:var(--critico); }
  .oruga .apagado { color:var(--apagado); }
  .botones { display:flex; flex-wrap:wrap; gap:6px; }
  button { font:inherit; font-size:12px; color:var(--tinta);
           background:var(--rejilla); border:1px solid var(--borde);
           border-radius:6px; padding:5px 10px; cursor:pointer; }
  button[aria-pressed="true"] { border-color:var(--s1); color:var(--s1); }
  button[hidden] { display:none; }
  .ayuda { font-size:11px; color:var(--apagado); margin-top:8px; }
  /* Mundo simulado: etiquetas y chip por debajo del mando (z-index 2); el
     chip va despues en el HTML para quedar encima de las etiquetas */
  #etiquetas { position:absolute; inset:0; z-index:1; pointer-events:none;
               overflow:hidden; }
  #etiquetas[hidden] { display:none; }
  .etiqueta { position:absolute; left:0; top:0; padding:1px 5px;
              border-radius:4px; font-size:11px; line-height:1.3;
              white-space:nowrap; color:var(--tinta2);
              background:rgba(13,13,13,.6); }
  #terreno { position:absolute; top:10px; left:50%; z-index:1;
             transform:translateX(-50%); max-width:calc(100% - 20px);
             width:max-content;   /* si no, left:50% lo corta a media escena */
             padding:4px 10px; border-radius:6px; background:var(--aviso);
             color:#0d0d0d; font-size:12px; font-weight:600;
             letter-spacing:.06em; text-align:center; pointer-events:none; }
  #terreno[hidden] { display:none; }
  #terreno.rojo { background:var(--critico); }
  #terreno.gris { background:var(--apagado); }
  #escena.estrecho #terreno { top:auto; bottom:10px; }
  #secTerreno[hidden] { display:none; }
  #secTerreno h2 { margin-top:14px; }
  .dato b.ambar { color:var(--aviso); }
  .dato b.rojo { color:var(--critico); }
__MANDO_CSS__
  @media (max-width:760px) {
    main { flex-direction:column; }
    aside { width:auto; }
  }
</style></head><body>

<header class="tarjeta cab">
  <h1>STARCRAWLER · VISTA 3D</h1>
  <span class="punto" id="enlace"></span>
  <span class="sub" id="info">esperando datos…</span>
  <a href="/" style="margin-left:auto">← Telemetría 2D</a>
</header>

<main>
  <div id="escena">
    <div id="sinEnlace" hidden>SIN ENLACE</div>
    <div id="etiquetas"></div>
    <div id="terreno" role="status" hidden></div>
    <div id="aviso">Cargando la vista…</div>
__MANDO_HTML__
  </div>
  <aside class="tarjeta">
    <h2 id="tPos">Posición (odom)</h2>
    <div class="dato"><span>x</span><b class="num" id="px">–</b></div>
    <div class="dato"><span>y</span><b class="num" id="py">–</b></div>
    <div class="dato"><span>rumbo</span><b class="num" id="pyaw">–</b></div>
    <div class="dato"><span>recorrido</span><b class="num" id="pdist">–</b></div>
    <h2>Velocidad</h2>
    <div class="dato"><span>lineal</span><b class="num" id="vlin">–</b></div>
    <div class="dato"><span>giro</span><b class="num" id="vang">–</b></div>
    <div id="secTerreno" hidden>
      <h2>Terreno</h2>
      <div class="dato"><span>estado</span><b id="tEstado">–</b></div>
      <div class="dato"><span>cabeceo</span><b class="num" id="tCabeceo">–</b></div>
      <div class="dato"><span>balanceo</span><b class="num" id="tBalanceo">–</b></div>
      <div class="dato"><span>altura del chasis</span><b class="num" id="tAltura">–</b></div>
      <div class="dato"><span>margen de apoyo</span><b class="num" id="tMargen">–</b></div>
      <div class="dato"><span>real / orugas</span><b class="num" id="tAvance">–</b></div>
      <div class="dato"><span>patinado</span><b class="num" id="tPatinado">–</b></div>
      <div class="dato"><span>error de odometría</span><b class="num" id="tErrOdom">–</b></div>
    </div>
    <h2>Orugas (elevación)</h2>
    <div id="orugas"></div>
    <h2>Cámara</h2>
    <div class="botones">
      <button id="bSeguir" aria-pressed="true">Seguir al robot</button>
      <button id="bPerfil" aria-pressed="false">Perfil</button>
      <button id="bDetras" aria-pressed="false">Detrás</button>
      <button id="bCenital">Cenital</button>
      <button id="bRastro">Borrar rastro</button>
      <button id="bSombras" aria-pressed="true" hidden>Sombras</button>
      <button id="bEtiquetas" aria-pressed="true" hidden>Etiquetas</button>
    </div>
    <p class="ayuda">Arrastrar: girar · Rueda o pellizco: zoom ·
      Mayús + arrastrar o botón derecho: desplazar</p>
    <div id="mAyuda" hidden>
      <h2 style="margin-top:14px">Mando (teclado)</h2>
      <p class="ayuda">WASD conducir · flechas inclinar · R/F delanteras ·
        T/G traseras · 1-4 presets (mantener) · Espacio EMERGENCIA ·
        Esc parar. Rearmar, solo con el ratón o el dedo.</p>
    </div>
  </aside>
</main>

<script>
__MANDO_JS__
</script>

<script>
  // Si three.js no llega (sin internet ni copia local), el modulo de abajo
  // no se ejecuta nunca: avisarlo en vez de quedarse cargando.
  setTimeout(() => {
    if (!window.vista3dLista) {
      document.getElementById('aviso').textContent =
        'No se pudo cargar three.js. Sin internet hace falta la copia ' +
        'local en starcrawler_gui/static/ (ver el README del paquete).';
    }
  }, 6000);
</script>

<script type="module">
import * as THREE from '__THREE_URL__';
window.vista3dLista = true;

const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const NOMBRE = ['FR', 'FL', 'RR', 'RL'];
const COLOR = ['--s1', '--s2', '--s3', '--s4'].map(css);
const JUNTA = ['crawler_fr_joint', 'crawler_fl_joint',
               'crawler_rr_joint', 'crawler_rl_joint'];
const $ = id => document.getElementById(id);

function aviso(texto) {
  $('aviso').hidden = !texto;
  if (texto) $('aviso').textContent = texto;
}

/* ── Escena: ROS usa Z arriba, asi que la camara tambien ──────────────── */
const contenedor = $('escena');
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(css('--pagina'));
// Solo hay sombras con mundo: las enciende sol.castShadow (aplicarSombras)
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
contenedor.prepend(renderer.domElement);

const escena = new THREE.Scene();
const camara = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
camara.up.set(0, 0, 1);

escena.add(new THREE.HemisphereLight(0xffffff, 0x303030, 2.2));
// El sol sigue al robot desde (3, -2, 6): la misma luz de siempre
const SOL = new THREE.Vector3(3, -2, 6);
const sol = new THREE.DirectionalLight(0xffffff, 1.8);
sol.position.copy(SOL);
const ladoSombra = window.innerWidth < 760 ? 1024 : 2048;
sol.shadow.mapSize.set(ladoSombra, ladoSombra);
Object.assign(sol.shadow.camera, {left: -4, right: 4, top: 4, bottom: -4, near: 0.5, far: 20});
sol.shadow.camera.updateProjectionMatrix();
sol.shadow.bias = -0.0005;
sol.shadow.normalBias = 0.01;
escena.add(sol, sol.target);

// Suelo de 20 x 20 m en celdas de 0,5 m, como plano.rviz
const rejilla = new THREE.GridHelper(20, 40, css('--eje'), css('--rejilla'));
rejilla.rotation.x = Math.PI / 2;
escena.add(rejilla);
escena.add(new THREE.AxesHelper(0.5));   // origen de odom: X rojo, Y verde
// Recibe las sombras sobre la rejilla
const sueloSombra = new THREE.Mesh(new THREE.PlaneGeometry(20, 20),
  new THREE.ShadowMaterial({opacity: 0.35, depthWrite: false}));
sueloSombra.position.z = 0.0005;
sueloSombra.receiveShadow = true;
sueloSombra.visible = false;
escena.add(sueloSombra);

/* ── Robot construido desde el URDF ───────────────────────────────────── */
// URDF rpy = Rz(yaw) Ry(pitch) Rx(roll), que en three.js es el orden ZYX
const cuaternion = rpy => new THREE.Quaternion().setFromEuler(
  new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));

function color(rgba) {
  return new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace);
}

function construir(m) {
  const links = {};
  for (const [nombre, visuales] of Object.entries(m.links)) {
    const grupo = new THREE.Group();
    grupo.name = nombre;
    for (const v of visuales) {
      let geo;
      if (v.tipo === 'caja') {
        geo = new THREE.BoxGeometry(v.tam[0], v.tam[1], v.tam[2]);
      } else if (v.tipo === 'cilindro') {
        geo = new THREE.CylinderGeometry(v.radio, v.radio, v.largo, 24);
        geo.rotateX(Math.PI / 2);          // URDF: eje Z; three.js: eje Y
      } else if (v.tipo === 'esfera') {
        geo = new THREE.SphereGeometry(v.radio, 20, 14);
      } else {
        continue;
      }
      const mat = new THREE.MeshStandardMaterial({
        color: color(v.color), roughness: 0.75, metalness: 0.05,
        transparent: v.color[3] < 1, opacity: v.color[3]});
      mat.userData.original = mat.color.clone();
      mat.userData.alfa = v.color[3];
      const malla = new THREE.Mesh(geo, mat);
      // Aristas claras: sin ellas el chasis oscuro se pierde en el fondo
      malla.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo),
        new THREE.LineBasicMaterial({color: 0xffffff, transparent: true, opacity: 0.3})));
      malla.position.fromArray(v.origen.xyz);
      malla.quaternion.copy(cuaternion(v.origen.rpy));
      malla.castShadow = true;
      grupo.add(malla);
    }
    links[nombre] = grupo;
  }

  const juntas = {};
  for (const j of m.joints) {
    if (!links[j.padre] || !links[j.hijo]) continue;
    const marco = new THREE.Group();       // origen fijo de la junta
    marco.position.fromArray(j.origen.xyz);
    marco.quaternion.copy(cuaternion(j.origen.rpy));
    const movil = new THREE.Group();       // lo que gira o desliza
    marco.add(movil);
    movil.add(links[j.hijo]);
    links[j.padre].add(marco);
    juntas[j.nombre] = {tipo: j.tipo, movil, hijo: links[j.hijo], periodo: j.periodo,
                        eje: new THREE.Vector3().fromArray(j.eje).normalize()};
  }
  return {raiz: links[m.raiz], juntas};
}

function moverJunta(j, q) {
  if (j.tipo === 'revolute' || j.tipo === 'continuous') {
    j.movil.quaternion.setFromAxisAngle(j.eje, q);
  } else if (j.tipo === 'prismatic') {
    j.movil.position.copy(j.eje).multiplyScalar(q);
  }
}

// Encoder caido: la oruga en gris translucido, como el trazo discontinuo
// de la vista 2D. Su angulo no es fiable.
function marcarEncoder(j, ok) {
  j.hijo.traverse(o => {
    if (!o.isMesh) return;
    const m = o.material;
    m.color.copy(ok ? m.userData.original : new THREE.Color(0x555555));
    m.opacity = m.userData.alfa * (ok ? 1 : 0.35);
    m.transparent = m.opacity < 1;
  });
}

let robot = null;

async function cargarModelo() {
  for (;;) {
    try {
      const r = await fetch('/modelo');
      if (r.ok) {
        const m = await r.json();
        robot = construir(m);
        escena.add(robot.raiz);
        aviso('');
        if (m.mallas_omitidas) {
          console.warn(m.mallas_omitidas + ' visuales con malla sin dibujar');
        }
        return;
      }
    } catch (e) { /* servidor reiniciando: reintentar */ }
    aviso('Esperando el modelo del robot (/robot_description). ' +
          '¿Está en marcha robot_state_publisher?');
    await new Promise(r => setTimeout(r, 2000));
  }
}

/* ── Rastros: el real y, con mundo, el fantasma de /odom ──────────────── */
const MAX_RASTRO = 20000;

function crearRastro(material, discontinuo) {
  const r = {pos: new Float32Array(MAX_RASTRO * 3), n: 0,
             dist: discontinuo ? new Float32Array(MAX_RASTRO) : null};
  r.geo = new THREE.BufferGeometry();
  r.geo.setAttribute('position', new THREE.BufferAttribute(r.pos, 3));
  // LineDashedMaterial necesita la distancia recorrida en cada vertice
  if (r.dist) r.geo.setAttribute('lineDistance', new THREE.BufferAttribute(r.dist, 1));
  r.geo.setDrawRange(0, 0);
  r.linea = new THREE.Line(r.geo, material);
  r.linea.frustumCulled = false;   // la esfera envolvente no se recalcula
  escena.add(r.linea);
  return r;
}

const rastro = crearRastro(new THREE.LineBasicMaterial({color: css('--s1')}), false);
const fantasma = crearRastro(new THREE.LineDashedMaterial(
  {color: css('--apagado'), dashSize: 0.05, gapSize: 0.04}), true);

function anadirRastro(r, x, y, z) {
  let d = 0;
  if (r.n > 0) {
    const i = (r.n - 1) * 3;
    d = Math.hypot(x - r.pos[i], y - r.pos[i + 1], z - r.pos[i + 2]);
    if (d < 0.02) return;
  }
  if (r.n >= MAX_RASTRO) {                 // lleno: se tira la mitad antigua
    r.pos.copyWithin(0, (MAX_RASTRO / 2) * 3);
    if (r.dist) r.dist.copyWithin(0, MAX_RASTRO / 2);
    r.n = MAX_RASTRO / 2;
  }
  r.pos.set([x, y, z], r.n * 3);
  if (r.dist) {
    r.dist[r.n] = r.n ? r.dist[r.n - 1] + d : 0;
    r.geo.attributes.lineDistance.needsUpdate = true;
  }
  r.n++;
  r.geo.setDrawRange(0, r.n);
  r.geo.attributes.position.needsUpdate = true;
}

function borrarRastro(r) { r.n = 0; r.geo.setDrawRange(0, 0); }

/* ── Mundo simulado: /mundo (fijo) y marcas (indicadores, en cada evento) */
const grupoMundo = new THREE.Group(), grupoMarcas = new THREE.Group();
escena.add(grupoMundo, grupoMarcas);
const capaEtiquetas = $('etiquetas');
const etiquetas = {mundo: [], marcas: []};             // [{el, ancla}]
const geoEsfera = new THREE.SphereGeometry(0.5, 16, 12);  // diametro 1
let hayMundo = false, sombras = true;
let mundoCargado = -1, pidiendoMundo = false, tFalloMundo = -1e9;
let marcasVistas = '';

// sRGB -> lineal, como color()
function lineal(colores) {
  const c = new THREE.Color(), out = new Float32Array(colores.length);
  for (let i = 0; i < colores.length; i += 4) {
    c.setRGB(colores[i], colores[i + 1], colores[i + 2], THREE.SRGBColorSpace);
    out[i] = c.r; out[i + 1] = c.g; out[i + 2] = c.b; out[i + 3] = colores[i + 3];
  }
  return out;
}

function alfaMinimo(colores) {
  let a = 1;
  for (let i = 3; i < colores.length; i += 4) a = Math.min(a, colores[i]);
  return a;
}

function transparencia(obj, alfa) {
  if (alfa >= 1) return;
  Object.assign(obj.material, {transparent: true, opacity: alfa, depthWrite: false});
  obj.renderOrder = 1;
}

const cssColor = c => `rgb(${c.slice(0, 3).map(v => Math.round(v * 255)).join(',')})`;

// Una pieza de mundo_modelo -> Object3D. Los textos van a la capa HTML.
function construirMarca(p, lista, conSombra) {
  const pieza = new THREE.Group();
  pieza.position.fromArray(p.pos);
  pieza.quaternion.set(p.quat[0], p.quat[1], p.quat[2], p.quat[3]);   // Z arriba, como ROS
  if (p.tipo === 'texto') {
    const el = document.createElement('div');
    el.className = 'etiqueta';
    el.textContent = p.texto;             // viene de ROS: nunca como HTML
    el.style.color = cssColor(p.color);
    el.style.visibility = 'hidden';
    capaEtiquetas.appendChild(el);
    lista.push({el, ancla: pieza});
    return pieza;
  }
  const conColores = !!p.colores;
  const base = conColores ? [1, 1, 1, alfaMinimo(p.colores)] : p.color;
  let obj;
  if (p.tipo === 'triangulos' || p.tipo === 'lineas' || p.tipo === 'tira') {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(p.puntos, 3));
    if (conColores) geo.setAttribute('color', new THREE.BufferAttribute(lineal(p.colores), 4));
    if (p.tipo === 'triangulos') {
      geo.computeVertexNormals();
      obj = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        color: color(base), roughness: 0.9, metalness: 0, side: THREE.DoubleSide,
        vertexColors: conColores,
        // Las aristas se dibujan encima sin parpadear
        polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1}));
      if (p.escala.every(v => v !== 0)) obj.scale.fromArray(p.escala);
      obj.castShadow = obj.receiveShadow = conSombra && base[3] >= 1;
    } else {
      const mat = new THREE.LineBasicMaterial({color: color(base), vertexColors: conColores});
      obj = p.tipo === 'lineas' ? new THREE.LineSegments(geo, mat) : new THREE.Line(geo, mat);
    }
    transparencia(obj, base[3]);
  } else if (p.tipo === 'esferas') {
    const n = p.puntos.length / 3;
    if (!n) return null;
    obj = new THREE.InstancedMesh(geoEsfera, new THREE.MeshStandardMaterial(
      {color: color(base), roughness: 0.9, metalness: 0}), n);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), v = new THREE.Vector3();
    const tam = new THREE.Vector3().fromArray(p.escala), c = new THREE.Color();
    for (let i = 0; i < n; i++) {
      obj.setMatrixAt(i, m.compose(v.fromArray(p.puntos, 3 * i), q, tam));
      if (conColores) {
        obj.setColorAt(i, c.setRGB(p.colores[4 * i], p.colores[4 * i + 1],
                                   p.colores[4 * i + 2], THREE.SRGBColorSpace));
      }
    }
    obj.frustumCulled = false;
    transparencia(obj, base[3]);
  } else if (p.tipo === 'flecha') {
    // RViz: escala = (diametro del asta, diametro de la punta, largo de la punta)
    const a = new THREE.Vector3().fromArray(p.puntos, 0);
    const d = new THREE.Vector3().fromArray(p.puntos, 3).sub(a);
    const largo = d.length();
    if (largo < 1e-6) return null;
    const punta = Math.min(p.escala[2] || 0.3 * largo, 0.9 * largo);
    obj = new THREE.ArrowHelper(d.normalize(), a, largo, color(p.color), punta,
                                p.escala[1] || 0.5 * punta);
  } else {
    return null;
  }
  pieza.add(obj);
  return pieza;
}

function vaciar(grupo, lista) {
  grupo.traverse(o => {
    if (o.material) o.material.dispose();
    // geoEsfera y la geometria de ArrowHelper son compartidas
    const compartida = o.geometry === geoEsfera || (o.parent && o.parent.type === 'ArrowHelper');
    if (o.geometry && !compartida) o.geometry.dispose();
    if (o.isInstancedMesh) o.dispose();
  });
  grupo.clear();
  for (const e of lista) e.el.remove();
  lista.length = 0;
}

async function cargarMundo() {
  pidiendoMundo = true;
  try {
    const r = await fetch('/mundo', {cache: 'no-store'});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const m = await r.json();
    vaciar(grupoMundo, etiquetas.mundo);
    for (const p of m.piezas) {
      const o = construirMarca(p, etiquetas.mundo, true);
      if (o) grupoMundo.add(o);
    }
    mundoCargado = m.version;
    hayMundo = m.piezas.length > 0;
    if (m.omitidos) console.warn(m.omitidos + ' marcadores del mundo sin dibujar');
    $('bSombras').hidden = !hayMundo;
    $('bEtiquetas').hidden = !hayMundo;
    aplicarSombras();
  } catch (e) {
    tFalloMundo = performance.now();     // servidor reiniciando: reintentar
  }
  pidiendoMundo = false;
}

function aplicarSombras() {
  const si = hayMundo && sombras;
  sueloSombra.visible = si;
  if (sol.castShadow === si) return;
  sol.castShadow = si;
  // Los materiales ya compilados no se enteran solos
  escena.traverse(o => { if (o.material) o.material.needsUpdate = true; });
}

function ponerMarcas(marcas) {
  const clave = marcas ? JSON.stringify(marcas) : '';
  if (clave === marcasVistas) return;
  marcasVistas = clave;
  vaciar(grupoMarcas, etiquetas.marcas);
  for (const p of marcas || []) {
    const o = construirMarca(p, etiquetas.marcas, false);
    if (o) grupoMarcas.add(o);
  }
  colocarEtiquetas(etiquetas.marcas);     // ya, para que no parpadeen
}

/* ── Etiquetas HTML: un ancla proyectada en cada fotograma ─────────────── */
const vAncla = new THREE.Vector3();

function colocarEtiquetas(lista) {
  if (capaEtiquetas.hidden) return;
  const w = contenedor.clientWidth, h = contenedor.clientHeight;
  for (const e of lista) {
    e.ancla.getWorldPosition(vAncla);
    const lejos = vAncla.distanceTo(camara.position) > 8;
    vAncla.project(camara);
    const oculta = lejos || vAncla.z > 1;          // lejos o detras de la camara
    e.el.style.visibility = oculta ? 'hidden' : 'visible';
    if (!oculta) {
      e.el.style.transform = `translate(${(vAncla.x + 1) / 2 * w}px, ` +
        `${(1 - vAncla.y) / 2 * h}px) translate(-50%, -50%)`;
    }
  }
}

/* ── Datos: el mismo /events que la vista 2D ──────────────────────────── */
// Llegan a 10 Hz; se suaviza hacia el ultimo valor para que no de saltos.
// El EventSource lo abre el script del mando: uno solo por pagina.
const Z_CENTRO = 0.1;                      // altura de la camara sin mundo
const objetivo = {x: 0, y: 0, yaw: 0, z: Z_CENTRO, juntas: {}};
const visto = {x: 0, y: 0, yaw: 0, z: Z_CENTRO, juntas: {}};
// Tacos de las orugas: la pagina integra la velocidad de banda en cada
// cuadro. Los de /joint_states (para RViz) van en diente de sierra y a
// 10 Hz el suavizado los haria ir y venir.
const BANDAS = [['crawler_fr', false], ['crawler_fl', true],
                ['crawler_rr', false], ['crawler_rl', true]];
const TACOS = ['arriba', 'abajo', 'polea', 'punta'];
const bandas = {vl: 0, vr: 0, t: 0, fase: [0, 0, 0, 0]};
const odomVista = {x: 0, y: 0};
let hayPose = false, recorrido = 0;

const difAngulo = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));

// Al reconectar puede ser otro gui_node: volver a pedir el mundo
window.estadoRobot.addEventListener('open', () => { mundoCargado = -1; });

window.estadoRobot.addEventListener('message', ev => {
  const s = JSON.parse(ev.data);
  const t = s.terreno;
  const zSuelo = t && typeof t.z_suelo === 'number' ? t.z_suelo : 0;
  // Con mundo, la pose verdadera; sin el, la de /odom como siempre
  const real = s.verdad || s.pose;
  if (real) {
    const [x, y, yaw] = real;
    const z = s.verdad ? s.verdad[5] : Z_CENTRO;
    const salto = Math.hypot(x - objetivo.x, y - objetivo.y);
    if (!hayPose || salto > 1.0) {
      // Primera pose, o la odometria se ha reiniciado: sin transicion
      Object.assign(visto, {x, y, yaw, z});
      if (hayPose) { borrarRastro(rastro); recorrido = 0; }
    } else {
      recorrido += salto;
    }
    Object.assign(objetivo, {x, y, yaw, z});
    hayPose = true;
    // Aqui y no en el bucle de dibujo: el navegador frena ese bucle con la
    // pestana en segundo plano y el rastro saldria a trozos rectos
    anadirRastro(rastro, x, y, zSuelo + 0.004);
  }
  fantasma.linea.visible = !!s.verdad;
  if (s.verdad && s.pose) {
    const [x, y] = s.pose;
    if (Math.hypot(x - odomVista.x, y - odomVista.y) > 1.0) borrarRastro(fantasma);
    Object.assign(odomVista, {x, y});
    anadirRastro(fantasma, x, y, zSuelo + 0.006);
  }
  if (s.joints) {
    for (const [n, q] of Object.entries(s.joints)) {
      if (!n.includes('_tacos_')) objetivo.juntas[n] = q;
    }
  }
  if (typeof s.vl === 'number' && typeof s.vr === 'number') {
    Object.assign(bandas, {vl: s.vl, vr: s.vr, t: performance.now()});
  }
  if (typeof s.mundo_v === 'number' && s.mundo_v !== mundoCargado && !pidiendoMundo &&
      performance.now() - tFalloMundo > 2000) {
    cargarMundo();
  }
  ponerMarcas(s.marcas);
  panel(s);
});

function fmt(v, dec, unidad) {
  return (v === null || v === undefined) ? '–' : v.toFixed(dec) + ' ' + unidad;
}

function panel(s) {
  $('enlace').className = 'punto' + (s.enlace ? ' on' : '');
  $('info').textContent = `fuente: ${s.fuente} · ${s.pps} paq/s` +
    (s.enlace ? '' : ' · SIN ENLACE') +
    ((s.err >> 7) & 1 ? ' · HARDWARE SIMULADO' : '');
  $('sinEnlace').hidden = !!s.enlace;

  const p = s.pose, r = s.verdad || s.pose;
  $('tPos').textContent = s.verdad ? 'Posición (real)' : 'Posición (odom)';
  $('px').textContent = r ? fmt(r[0], 2, 'm') : 'sin /odom';
  $('py').textContent = r ? fmt(r[1], 2, 'm') : '–';
  $('pyaw').textContent = r ? fmt(r[2] * 180 / Math.PI, 0, '°') : '–';
  $('pdist').textContent = r ? fmt(recorrido, 2, 'm') : '–';
  $('vlin').textContent = p ? fmt(p[3], 2, 'm/s') : '–';
  $('vang').textContent = p ? fmt(p[4] * 180 / Math.PI, 1, '°/s') : '–';

  const t = s.terreno;
  $('orugas').innerHTML = JUNTA.map((nombre, i) => {
    const q = s.joints ? s.joints[nombre] : undefined;
    const ok = !((s.err >> i) & 1);
    const ang = (q === undefined) ? '–' : (q >= 0 ? '+' : '') + (q * 180 / Math.PI).toFixed(1) + '°';
    return `<div class="oruga"><i style="background:${COLOR[i]}"></i><b>${NOMBRE[i]}</b>` +
      `<span class="num">${ang}</span>` + apoya(t, i) +
      `<span class="${ok ? '' : 'mal'}">${ok ? '✓' : '✕ enc'}</span></div>`;
  }).join('');

  if (robot) {
    JUNTA.forEach((nombre, i) => {
      if (robot.juntas[nombre]) marcarEncoder(robot.juntas[nombre], !((s.err >> i) & 1));
    });
  }
  pintarTerreno(s);
}

// Columna 'apoya' de #orugas: solo numeros, nada de texto de ROS
function apoya(t, i) {
  if (!t || !Array.isArray(t.apoya)) return '';
  if (t.apoya[i]) return '<span title="apoya">●</span>';
  const h = Array.isArray(t.holgura) ? t.holgura[i] : null;
  const cm = typeof h === 'number' ? ' ' + (h * 100).toFixed(1) + ' cm' : '';
  return `<span class="num apagado" title="no apoya">○${cm}</span>`;
}

/* ── Terreno: chip en la escena y seccion del panel ───────────────────── */
const ESTADOS = {libre: 'libre', bloqueado: 'bloqueado', sin_traccion: 'sin tracción',
                 cayendo: 'cayendo', atrapado: 'atrapado', volcado: 'volcado'};

// '' ambar, 'rojo' o 'gris'; null si no hay nada que avisar
function claseTerreno(t) {
  if (t.modo === 'espejo') return 'gris';
  if (!t.estado || t.estado === 'libre') return null;
  if (t.estado === 'volcado') return 'rojo';
  return ESTADOS[t.estado] ? '' : 'gris';
}

function pintarTerreno(s) {
  const t = s.terreno, conMundo = s.mundo_v > 0 || !!t;
  let partes = null, clase = null;
  if (t) {
    clase = claseTerreno(t);
    const nombre = t.modo === 'espejo' ? 'espejo' : (ESTADOS[t.estado] || String(t.estado));
    if (clase !== null) partes = [nombre.toUpperCase(), t.motivo, t.consejo];
  } else if (conMundo) {
    partes = ['terreno: sin datos'];
    clase = 'gris';
  }
  const chip = $('terreno');
  chip.hidden = !partes;
  if (partes) {
    const texto = partes.filter(x => x).join(' · ');
    if (chip.textContent !== texto) chip.textContent = texto;
    chip.className = clase;
    colocarChip();
  }

  $('secTerreno').hidden = !conMundo;
  if (!conMundo) return;
  const d = t || {};
  const n = (v, dec) => (typeof v === 'number' ? v.toFixed(dec) : '–');
  const est = !t ? 'sin datos'
    : (t.modo === 'espejo' ? 'espejo' : (ESTADOS[t.estado] || String(t.estado || '–'))) +
      (t.pieza ? ' · ' + t.pieza : '');
  const clEst = t ? claseTerreno(t) : 'gris';
  poner('tEstado', est, clEst === 'rojo' ? 'rojo' : clEst === '' ? 'ambar' : '');
  angulo('tCabeceo', d.cabeceo);
  angulo('tBalanceo', d.balanceo);
  poner('tAltura', fmt(d.altura, 3, 'm'), 'num');
  const mg = d.margen;
  poner('tMargen', fmt(mg, 3, 'm'), 'num' + (typeof mg !== 'number' ? '' :
        mg < 0.03 ? ' rojo' : mg < 0.10 ? ' ambar' : ''));
  poner('tAvance', t ? `${n(d.avance_real, 3)} / ${n(d.avance_orugas, 3)} m/s` : '–', 'num');
  poner('tPatinado', fmt(d.patinado, 2, 'm'), 'num');
  const v = s.verdad, p = s.pose;
  poner('tErrOdom', v && p ? fmt(Math.hypot(v[0] - p[0], v[1] - p[1]), 3, 'm') : '–', 'num');
}

function poner(id, texto, clase) {
  const el = $(id);
  if (el.textContent !== texto) el.textContent = texto;
  if (el.className !== clase) el.className = clase;
}

// En grados, ambar por encima de 25 y rojo por encima de 35
function angulo(id, rad) {
  if (typeof rad !== 'number') { poner(id, '–', 'num'); return; }
  const d = rad * 180 / Math.PI, a = Math.abs(d);
  poner(id, (d >= 0 ? '+' : '') + d.toFixed(1) + '°',
        'num' + (a > 35 ? ' rojo' : a > 25 ? ' ambar' : ''));
}

// Sin tapar el mando: por debajo de su fila de arriba o, en estrecho, por
// encima de sus controles
function colocarChip() {
  const chip = $('terreno');
  const alto = el => { const h = el && !el.hidden ? el.offsetHeight : 0; return h ? h + 8 : 0; };
  if (contenedor.classList.contains('estrecho')) {
    chip.style.top = '';
    chip.style.bottom = 10 + Math.max(alto($('mAbajo')), alto($('mLinea'))) + 'px';
  } else {
    chip.style.bottom = '';
    chip.style.top = 10 + alto(document.querySelector('#mando .m-arriba')) + 'px';
  }
}

/* ── Camara orbital ───────────────────────────────────────────────────── */
const orbita = {az: -2.3, el: 0.55, dist: 4, centro: new THREE.Vector3(0, 0, Z_CENTRO),
                seguir: true, vista: ''};
// Perfil y Detras siguen el rumbo mientras estan pulsados
const VISTAS = {perfil: {boton: 'bPerfil', az: Math.PI / 2, el: 0.12},
                detras: {boton: 'bDetras', az: Math.PI, el: 0.35}};
const punteros = new Map();
let pellizco = 0;

function colocarCamara() {
  const v = VISTAS[orbita.vista];
  if (v) { orbita.az = visto.yaw + v.az; orbita.el = v.el; }
  // visto.z: la z de base_link con mundo, Z_CENTRO sin el
  if (orbita.seguir) orbita.centro.set(visto.x, visto.y, visto.z);
  const c = Math.cos(orbita.el);
  camara.position.set(
    orbita.centro.x + orbita.dist * c * Math.cos(orbita.az),
    orbita.centro.y + orbita.dist * c * Math.sin(orbita.az),
    orbita.centro.z + orbita.dist * Math.sin(orbita.el));
  camara.lookAt(orbita.centro);
}

function fijarVista(nombre) {
  orbita.vista = nombre;
  for (const [n, v] of Object.entries(VISTAS)) {
    $(v.boton).setAttribute('aria-pressed', String(n === nombre));
  }
  if (nombre) seguir(true);
}

function seguir(si) {
  orbita.seguir = si;
  $('bSeguir').setAttribute('aria-pressed', String(si));
  if (!si) fijarVista('');
}

function desplazar(dx, dy) {
  seguir(false);
  // Sobre el suelo, en los ejes de la pantalla
  const k = orbita.dist * 0.0016;
  const derecha = new THREE.Vector3(-Math.sin(orbita.az), Math.cos(orbita.az), 0);
  const fondo = new THREE.Vector3(-Math.cos(orbita.az), -Math.sin(orbita.az), 0);
  orbita.centro.addScaledVector(derecha, -dx * k).addScaledVector(fondo, dy * k);
}

const lienzo = renderer.domElement;
lienzo.addEventListener('contextmenu', e => e.preventDefault());
lienzo.addEventListener('pointerdown', e => {
  lienzo.setPointerCapture(e.pointerId);
  punteros.set(e.pointerId, {x: e.clientX, y: e.clientY});
});
lienzo.addEventListener('pointermove', e => {
  const antes = punteros.get(e.pointerId);
  if (!antes) return;
  const ahora = {x: e.clientX, y: e.clientY};
  punteros.set(e.pointerId, ahora);
  if (punteros.size === 2) {               // pellizco
    const [a, b] = [...punteros.values()];
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    if (pellizco) orbita.dist = Math.min(40, Math.max(0.6, orbita.dist * pellizco / d));
    pellizco = d;
    return;
  }
  const dx = ahora.x - antes.x, dy = ahora.y - antes.y;
  if (e.shiftKey || e.buttons === 2) {
    desplazar(dx, dy);
  } else {
    fijarVista('');
    orbita.az -= dx * 0.008;
    orbita.el = Math.min(1.55, Math.max(0.05, orbita.el + dy * 0.008));
  }
});
const soltar = e => { punteros.delete(e.pointerId); if (punteros.size < 2) pellizco = 0; };
lienzo.addEventListener('pointerup', soltar);
lienzo.addEventListener('pointercancel', soltar);
lienzo.addEventListener('wheel', e => {
  e.preventDefault();
  orbita.dist = Math.min(40, Math.max(0.6, orbita.dist * Math.exp(e.deltaY * 0.001)));
}, {passive: false});

$('bSeguir').onclick = () => seguir(!orbita.seguir);
$('bPerfil').onclick = () => fijarVista(orbita.vista === 'perfil' ? '' : 'perfil');
$('bDetras').onclick = () => fijarVista(orbita.vista === 'detras' ? '' : 'detras');
$('bCenital').onclick = () => { fijarVista(''); orbita.el = 1.55; orbita.az = -Math.PI / 2; };
$('bRastro').onclick = () => { borrarRastro(rastro); borrarRastro(fantasma); recorrido = 0; };
$('bSombras').onclick = () => {
  sombras = !sombras;
  $('bSombras').setAttribute('aria-pressed', String(sombras));
  aplicarSombras();
};
$('bEtiquetas').onclick = () => {
  capaEtiquetas.hidden = !capaEtiquetas.hidden;
  $('bEtiquetas').setAttribute('aria-pressed', String(!capaEtiquetas.hidden));
};

new ResizeObserver(() => {
  const w = contenedor.clientWidth, h = contenedor.clientHeight;
  renderer.setSize(w, h, false);
  camara.aspect = w / Math.max(1, h);
  camara.updateProjectionMatrix();
}).observe(contenedor);

/* ── Bucle de dibujo ──────────────────────────────────────────────────── */
let tAnterior = performance.now();
function bucle(t) {
  const dt = Math.min(0.1, (t - tAnterior) / 1000);
  tAnterior = t;
  const k = 1 - Math.exp(-dt / 0.08);

  visto.x += (objetivo.x - visto.x) * k;
  visto.y += (objetivo.y - visto.y) * k;
  visto.yaw += difAngulo(objetivo.yaw, visto.yaw) * k;
  // Mas lenta: que la camara no pegue saltos en los cantos
  visto.z += (objetivo.z - visto.z) * (1 - Math.exp(-dt / 0.3));
  for (const [n, q] of Object.entries(objetivo.juntas)) {
    const antes = visto.juntas[n] ?? q;
    visto.juntas[n] = antes + (q - antes) * k;
  }

  if (robot) {
    // En z = 0: la altura, el cabeceo y el balanceo van en las juntas
    robot.raiz.position.set(visto.x, visto.y, 0);
    robot.raiz.rotation.set(0, 0, visto.yaw);
    for (const [n, q] of Object.entries(visto.juntas)) {
      if (robot.juntas[n]) moverJunta(robot.juntas[n], q);
    }
    // Fase en pasos de taco: el arco de la polea activa tiene un taco por
    // periodo, asi que w / periodo son pasos por segundo
    const fresca = t - bandas.t < 500;
    BANDAS.forEach(([b, izq], i) => {
      const polea = robot.juntas[b + '_tacos_polea_joint'];
      if (!polea || !polea.periodo) return;
      const dps = fresca ? (izq ? bandas.vl : bandas.vr) : 0;
      bandas.fase[i] += dps * Math.PI / 180 * dt / polea.periodo;
      bandas.fase[i] -= Math.floor(bandas.fase[i]);
      for (const lado of TACOS) {
        const j = robot.juntas[b + '_tacos_' + lado + '_joint'];
        if (j && j.periodo) moverJunta(j, bandas.fase[i] * j.periodo);
      }
    });
  }
  sol.target.position.set(visto.x, visto.y, 0);
  sol.position.copy(sol.target.position).add(SOL);
  colocarCamara();
  renderer.render(escena, camara);
  colocarEtiquetas(etiquetas.mundo);
  colocarEtiquetas(etiquetas.marcas);
  requestAnimationFrame(bucle);
}

aviso('');
cargarModelo();
requestAnimationFrame(bucle);
</script></body></html>
"""


# ─── Mando web ──────────────────────────────────────────────────────────────

MANDO_CSS = r"""
  /* ── Mando web: encima de la escena; solo los controles cogen el puntero */
  #mando { position:absolute; inset:0; z-index:2; pointer-events:none;
           display:flex; flex-direction:column; justify-content:space-between;
           gap:10px; padding:10px; }
  #mando [hidden] { display:none !important; }
  body.con-mando #sinEnlace { display:none; }   /* lo dice la tira */
  body.con-mando #escena { min-height:460px; }
  body.con-mando #escena.estrecho { min-height:620px; }
  .m-arriba { display:flex; justify-content:space-between;
              align-items:flex-start; gap:10px; }
  #escena.estrecho .m-arriba { flex-direction:column-reverse;
                               align-items:stretch; gap:8px; }
  .m-tira, .m-linea { padding:6px 10px; border-radius:8px; font-size:12px;
                      line-height:1.5; color:var(--tinta2);
                      background:rgba(13,13,13,.78); border:1px solid var(--borde); }
  .m-tira { max-width:440px; }
  .m-tira b { font-weight:600; color:var(--tinta); }
  .m-linea { align-self:flex-start; }
  #mando .e-ok { color:var(--ok); }
  #mando .e-aviso { color:var(--aviso); }
  #mando .e-mal { color:var(--critico); }
  #mEmer { margin-left:auto; display:flex; flex-direction:column;
           align-items:flex-end; gap:4px; }
  #mEmerEstado { font-size:12px; font-weight:700; letter-spacing:.04em;
                 text-align:right; text-shadow:0 1px 3px #000; }

  .m-abajo { display:grid; gap:10px; align-items:end;
             grid-template-columns:1fr auto auto auto;
             grid-template-areas:". acc acc acc" "pad par cru pre"; }
  #escena.estrecho #mando { gap:8px; padding:8px; }
  #escena.estrecho .m-abajo { gap:8px; grid-template-columns:auto auto;
             justify-content:space-between;
             grid-template-areas:"acc acc" "par cru" "pad pre"; }
  #mPad { grid-area:pad; justify-self:start; }
  #mAcciones { grid-area:acc; justify-self:end; display:flex; gap:8px; }
  #mPares { grid-area:par; }
  #mCruceta { grid-area:cru; justify-self:end; }
  #mPresets { grid-area:pre; justify-self:end; }

  #mando .control { pointer-events:auto; touch-action:none; user-select:none;
    -webkit-user-select:none; -webkit-touch-callout:none;
    -webkit-tap-highlight-color:transparent; min-width:48px; min-height:48px;
    position:relative; overflow:hidden; display:flex; flex-direction:column;
    align-items:center; justify-content:center; gap:1px; padding:0 10px;
    color:var(--tinta); background:rgba(26,26,25,.86); line-height:1.1;
    border:1px solid rgba(255,255,255,.18); border-radius:8px; }
  #mando .control b { position:relative; font-size:17px; font-weight:600; }
  #mando .control small { position:relative; font-size:10px; color:var(--tinta2); }
  #mando .control.on { border-color:var(--s1); background:rgba(57,135,229,.32); }
  #mando .relleno { position:absolute; inset:0; background:rgba(57,135,229,.45);
                    transform:scaleX(0); transform-origin:left; }
  #mando .preset.enganchado { border-color:var(--s1); }
  #mando .accion { flex-direction:row; gap:6px; padding:0 14px; font-size:13px; }
  #mando .accion b { font-size:13px; }
  #mando .emergencia { min-width:132px; min-height:56px; font-size:15px;
    font-weight:700; letter-spacing:.08em; background:var(--critico);
    border-color:rgba(255,255,255,.45); }
  #mando .emergencia.on { border-color:var(--tinta); }

  .m-grupo { display:grid; gap:4px; }
  #mPares { grid-template-columns:repeat(2,56px);
            grid-template-rows:auto repeat(2,56px); }
  #mPares .cap { font-size:10px; color:var(--apagado); text-align:center;
                 text-transform:uppercase; letter-spacing:.08em; }
  .m-rombo { grid-template-columns:repeat(3,48px); grid-template-rows:repeat(3,48px); }
  .m-rombo > .a { grid-area:1/2; }  .m-rombo > .i { grid-area:2/1; }
  .m-rombo > .d { grid-area:2/3; }  .m-rombo > .b { grid-area:3/2; }
  .m-rombo > span { grid-area:2/2; align-self:center; text-align:center;
                    font-size:10px; color:var(--apagado); }

  #mando #mPad { width:144px; height:144px; padding:0; display:block;
    overflow:visible; border-radius:50%; border-color:rgba(255,255,255,.22);
    background:radial-gradient(circle closest-side, rgba(13,13,13,.7) 0 10%,
               rgba(26,26,25,.6) 10.5% 100%); }
  #escena.estrecho #mando #mPad { width:128px; height:128px; }
  #mPadBola { position:absolute; left:50%; top:50%; width:52px; height:52px;
              margin:-26px 0 0 -26px; border-radius:50%; pointer-events:none;
              background:var(--eje); border:1px solid var(--tinta2); }
  #mPad.on #mPadBola { background:var(--s1); border-color:var(--tinta); }
"""

MANDO_HTML = r"""
    <div id="mando" data-modo="espera">
      <div class="m-arriba">
        <div id="mTira" class="m-tira" hidden>
          <div>Manda: <b id="mManda">–</b> · Robot: <b id="mRobot">–</b></div>
          <div><span id="mEnlPC">–</span> · <span id="mEnlESP">–</span><span
            id="mSinMux" class="e-mal" hidden> · sin mux</span></div>
          <div class="num" id="mVel">–</div>
          <div id="mPreset" hidden></div>
          <div id="mMotivo" class="e-aviso" hidden></div>
        </div>
        <div id="mEmer" hidden>
          <button class="control emergencia" data-c="emergencia" tabindex="-1"
                  title="Espacio">EMERGENCIA</button>
          <span id="mEmerEstado"></span>
        </div>
      </div>
      <div id="mLinea" class="m-linea" hidden></div>
      <div id="mAbajo" class="m-abajo" hidden>
        <div id="mPad" class="control" data-c="pad" tabindex="-1"
             role="application" aria-label="Conducir (WASD)"><div id="mPadBola"></div></div>
        <div id="mAcciones">
          <button class="control accion" data-c="parar" tabindex="-1"
                  title="Esc">Parar</button>
          <button class="control accion" data-c="lento" tabindex="-1"
                  aria-pressed="false" title="Velocidad lenta, como L3">Lento</button>
          <button class="control accion" id="mRearmar" data-c="rearmar" tabindex="-1"
                  title="Mantener 1 s" hidden><span class="relleno"></span><b>Rearmar</b><small>mantener 1 s</small></button>
        </div>
        <div id="mPares" class="m-grupo">
          <span class="cap">del.</span><span class="cap">tras.</span>
          <button class="control" data-c="l1" tabindex="-1" title="L1 · tecla R"><b>▲</b><small>L1</small></button>
          <button class="control" data-c="r1" tabindex="-1" title="R1 · tecla T"><b>▲</b><small>R1</small></button>
          <button class="control" data-c="l2" tabindex="-1" title="L2 · tecla F"><b>▼</b><small>L2</small></button>
          <button class="control" data-c="r2" tabindex="-1" title="R2 · tecla G"><b>▼</b><small>R2</small></button>
        </div>
        <div id="mCruceta" class="m-grupo m-rombo">
          <button class="control a" data-c="arriba" tabindex="-1" title="Inclinar adelante · flecha arriba"><b>▲</b></button>
          <button class="control i" data-c="izq" tabindex="-1" title="Inclinar a la izquierda · flecha izquierda"><b>◀</b></button>
          <span>inclinar</span>
          <button class="control d" data-c="der" tabindex="-1" title="Inclinar a la derecha · flecha derecha"><b>▶</b></button>
          <button class="control b" data-c="abajo" tabindex="-1" title="Inclinar atrás · flecha abajo"><b>▼</b></button>
        </div>
        <div id="mPresets" class="m-grupo m-rombo">__BOTONES_PRESET__<span>preset</span></div>
      </div>
    </div>
"""

_MANDO_JS = r"""
// Mando web (issue #18). Hombre muerto: solo manda mientras hay algo pulsado
// o un preset enganchado y, ante cualquier duda, lo suelta todo.
window.estadoRobot = new EventSource('/events');

(() => {
'use strict';

const PRESETS_DEG = __PRESETS_DEG__;
const SIMBOLOS = __SIMBOLOS__;
const S_PRESET = __S_PRESET__;
const NOMBRE = ['FR', 'FL', 'RR', 'RL'];
const JUNTA = ['crawler_fr_joint', 'crawler_fl_joint',
               'crawler_rr_joint', 'crawler_rl_joint'];
// Como acaba un preset segun /events (mando_web)
const TERMINALES = ['llegado', 'cancelado', 'sin encoder', 'sin enlace', 'tiempo'];

const PERIODO_MS = 50;        // bucle de envio, 20 Hz
const HUECO_MS = 250;
const NEUTRO_MS = 250;
const PLAZO_MS = 300;
// Por debajo de los 0,2 s de frescura del servidor: si /mando tarda mas, que
// se note aqui en vez de caducar alli cada orden
const PLAZO_MANDO_MS = 180;
// Sin keydown de autorrepeticion en este tiempo, la tecla se da por soltada
const TECLA_MAX_MS = 2500;
const SIN_EVENTOS_MS = 1000;
const EMER_CADA_MS = 100;
const EMER_AVISO_MS = 2000;
const REARME_MS = 1000;
const MOTIVO_MS = 6000;
const ZONA_MUERTA = 0.1;

// Lo que cuenta como "algo pulsado"
const MUEVE = new Set(['pad', 'w', 'a', 's', 'd', 'l1', 'l2', 'r1', 'r2',
                       'arriba', 'abajo', 'izq', 'der', 'p0', 'p1', 'p2', 'p3']);
// e.code -> control. Rearmar nunca por teclado.
const TECLAS = {
  KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd',
  ArrowUp: 'arriba', ArrowDown: 'abajo', ArrowLeft: 'izq', ArrowRight: 'der',
  KeyR: 'l1', KeyF: 'l2', KeyT: 'r1', KeyG: 'r2',
  Digit1: 'p0', Digit2: 'p1', Digit3: 'p2', Digit4: 'p3',
  Space: 'emergencia', Escape: 'parar',
};
const MANDA_OTRO = {joy_activo: 'MANDO FÍSICO', emergencia: 'EMERGENCIA WEB'};

const $ = id => document.getElementById(id);
const escena = $('escena'), mando = $('mando');
const pad = $('mPad'), bola = $('mPadBola'), bRearmar = $('mRearmar');
const controles = Array.from(mando.querySelectorAll('.control'));
const bPreset = PRESETS_DEG.map((_, k) => mando.querySelector(`[data-c="p${k}"]`));

/* ── Clave (fragmento #t= del enlace del log) y sesion ─────────────────── */
const GUARDADA = 'starcrawler_clave';

function leerClave() {
  const m = /[#&]t=([^&]*)/.exec(location.hash);
  if (!m) {
    try { return sessionStorage.getItem(GUARDADA) || ''; } catch (e) { return ''; }
  }
  let c = '';
  try { c = decodeURIComponent(m[1]).trim(); } catch (e) { /* mal escrita */ }
  try { sessionStorage.setItem(GUARDADA, c); } catch (e) { /* sin almacenamiento */ }
  // Fuera de la barra de direcciones y del historial
  try { history.replaceState(null, '', '/3d'); } catch (e) { /* nada */ }
  return c;
}

let clave = leerClave();

const sesion = Array.from(crypto.getRandomValues(new Uint8Array(12)),
                          b => b.toString(16).padStart(2, '0')).join('');

/* ── Estado ────────────────────────────────────────────────────────────── */
const punteros = new Map();   // pointerId -> {c, t0, x, y} | {muerto: true}
const teclas = new Set();
const vistaTecla = new Map(); // tecla -> ultimo keydown, con los de repeticion
let lento = false;
let preset = null;            // {k, t0, enganchado, tVisto}
let ultimo = null, tEvento = 0;
let tTick = 0, tNeutroHasta = 0;
let seq = 0, enVuelo = false, pendiente = null;
let emer = null;              // {t0, confirmada}
let rearme = null;            // {t0}
let motivo = '', tMotivo = 0;
let modoPintado = '';

function modo() {
  if (!ultimo) return 'espera';
  if (!ultimo.mando || !ultimo.mando.habilitado) return 'off';
  return clave ? 'activo' : 'bloqueado';
}
const activa = () => (ultimo && ultimo.mando && ultimo.mando.activa) || '';
const sPreset = () => (ultimo && ultimo.mando && ultimo.mando.s_preset) || S_PRESET;
const frescos = t => t - tEvento < SIN_EVENTOS_MS;
const limitar = v => Math.max(-1, Math.min(1, v));
const redondear = v => Math.round(v * 1000) / 1000;
const grados = d => (Math.round(d) > 0 ? '+' : '') + Math.round(d) + '°';
const nombrePreset = k => SIMBOLOS[k] + ' ' + grados(PRESETS_DEG[k]);
const presetDe = c => (/^p[0-9]$/.test(c) ? Number(c[1]) : -1);

function avisar(texto) { motivo = texto; tMotivo = performance.now(); }

function olvidarClave() {
  clave = '';
  try { sessionStorage.removeItem(GUARDADA); } catch (e) { /* nada */ }
  aplicarModo();
}

/* ── Lo pulsado -> orden ───────────────────────────────────────────────── */
function zonaMuerta(x, y) {
  const m = Math.hypot(x, y);
  if (m < ZONA_MUERTA) return [0, 0];
  const k = (Math.min(m, 1) - ZONA_MUERTA) / (1 - ZONA_MUERTA) / m;
  return [x * k, y * k];
}

function leer() {
  const on = new Set(teclas);
  let x = 0, y = 0;
  for (const p of punteros.values()) {
    if (p.muerto) continue;
    on.add(p.c);
    if (p.c === 'pad') { const v = zonaMuerta(p.x, p.y); x += v[0]; y += v[1]; }
  }
  const b = c => (on.has(c) ? 1 : 0);
  const sentidos = ['arriba', 'abajo', 'izq', 'der'].filter(c => on.has(c));
  return {
    on,
    avance: redondear(limitar(y + b('w') - b('s'))),
    giro: redondear(limitar(x + b('d') - b('a'))),      // + = derecha
    delanteras: b('l1') - b('l2'),
    traseras: b('r1') - b('r2'),
    // Dos sentidos a la vez no inclinan, como la cruceta del DS4
    inclinar: sentidos.length === 1 ? sentidos[0] : null,
  };
}

function hayMando(l) {
  for (const c of l.on) if (MUEVE.has(c)) return true;
  return !!(preset && preset.enganchado);
}

function orden(l) {
  return {avance: l.avance, giro: l.giro, lento, delanteras: l.delanteras,
          traseras: l.traseras, inclinar: l.inclinar,
          preset: preset ? preset.k : null};
}

function neutro() {
  return {avance: 0, giro: 0, lento, delanteras: 0, traseras: 0,
          inclinar: null, preset: null};
}

const esNeutra = o => !o.avance && !o.giro && !o.delanteras && !o.traseras &&
                      o.inclinar === null && o.preset === null;

// Un dedo que siga apoyado no cuenta hasta que se levante
function soltarTodo(por, conNeutro = true) {
  const habia = hayMando(leer());
  for (const id of punteros.keys()) punteros.set(id, {muerto: true});
  teclas.clear();
  preset = null;
  pendiente = null;
  if (!habia) return;
  if (por) avisar(por);
  if (conNeutro) {
    tNeutroHasta = performance.now() + NEUTRO_MS;
    enviar(neutro());
  }
}

/* ── Envio: una peticion en vuelo y, como mucho, la ultima pendiente ───── */
async function enviar(o) {
  if (enVuelo) { pendiente = o; return; }
  enVuelo = true;
  const neutra = esNeutra(o);
  const ctl = new AbortController();
  const plazo = setTimeout(() => ctl.abort(), PLAZO_MANDO_MS);
  let fallo = '';
  try {
    const r = await fetch('/mando', {
      method: 'POST', cache: 'no-store', signal: ctl.signal,
      headers: {'X-StarCrawler-Token': clave, 'Content-Type': 'application/json'},
      body: JSON.stringify(Object.assign({sesion, seq: seq++}, o)),
    });
    let j = {};
    try { j = (await r.json()) || {}; } catch (e) { if (e.name === 'AbortError') throw e; }
    if (r.status === 401) olvidarClave();
    if (!r.ok || !j.ok) fallo = j.motivo || ('HTTP ' + r.status);
  } catch (e) {
    fallo = e.name === 'AbortError' ? 'sin respuesta en ' + PLAZO_MANDO_MS + ' ms' : 'sin red';
  }
  clearTimeout(plazo);
  enVuelo = false;
  if (fallo) {
    pendiente = null;
    if (neutra) tNeutroHasta = 0;     // no insistir con el neutro
    soltarTodo('');
    avisar('SIN MANDO · ' + fallo);
    return;
  }
  if (pendiente) { const p = pendiente; pendiente = null; enviar(p); }
}

/* ── Emergencia y rearme ───────────────────────────────────────────────── */
function emergencia() {
  soltarTodo('', false);
  tNeutroHasta = 0;
  if (!emer) emer = {t0: performance.now(), confirmada: activa() === 'emergencia'};
  pedirEmergencia();
  pintarTira(performance.now());
}

// Sin clave: cualquiera puede parar
function pedirEmergencia() {
  const ctl = new AbortController();
  const plazo = setTimeout(() => ctl.abort(), PLAZO_MS);
  fetch('/emergencia', {method: 'POST', cache: 'no-store', signal: ctl.signal})
    .catch(() => { /* se repite hasta verla en /events */ })
    .finally(() => clearTimeout(plazo));
}

function rearmar() {
  rearme = {t0: performance.now()};
  const ctl = new AbortController();
  const plazo = setTimeout(() => ctl.abort(), PLAZO_MS);
  fetch('/rearmar', {method: 'POST', cache: 'no-store', signal: ctl.signal,
                     headers: {'X-StarCrawler-Token': clave}})
    .then(r => {
      if (r.status === 401) olvidarClave();
      if (!r.ok) { rearme = null; avisar('rearme rechazado · HTTP ' + r.status); }
    })
    .catch(() => { rearme = null; avisar('rearme: sin respuesta del PC'); })
    .finally(() => clearTimeout(plazo));
}

/* ── Presets: mantener s_preset para engancharlo ───────────────────────── */
function pulsarPreset(k) {
  if (preset && preset.k === k && preset.enganchado) return;
  preset = {k, t0: performance.now(), enganchado: false, tVisto: 0};
}

function terminarPreset(por) {
  if (preset) avisar('preset ' + nombrePreset(preset.k) + ': ' + por);
  preset = null;
}

function seguirPreset(p, t) {
  if (!preset || !preset.enganchado || !p || p.k !== preset.k) return;
  if (TERMINALES.includes(p.estado)) terminarPreset(p.estado);
  // 'esperando' no lo mantiene vivo: si el PC no lo arranca en 1 s, fuera
  else if (p.estado === 'en curso') preset.tVisto = t;
}

function bloqueo(t) {
  if (modo() !== 'activo') return 'mando bloqueado';
  if (!frescos(t)) return 'SIN CONEXIÓN CON EL PC';
  if (emer || activa() === 'emergencia') return 'EMERGENCIA: hay que rearmar';
  if (activa() === 'joy_activo') return 'manda el MANDO FÍSICO';
  return '';
}

/* ── Bucle a 20 Hz ─────────────────────────────────────────────────────── */
function tick() {
  const t = performance.now();
  const hueco = tTick && t - tTick > HUECO_MS;
  tTick = t;
  if (hueco) soltarTodo('pausa del navegador');
  if (!frescos(t)) soltarTodo('SIN CONEXIÓN CON EL PC');
  if (rearme && t - rearme.t0 > EMER_AVISO_MS) {
    rearme = null;
    avisar('rearme no aceptado: ¿emergencia fresca (SHARE)?');
  }
  for (const p of punteros.values()) {
    if (p.c === 'rearmar' && !p.hecho && t - p.t0 >= REARME_MS) { p.hecho = true; rearmar(); }
  }
  // Un keyup perdido no deja una tecla pulsada: la autorrepeticion del SO la
  // renueva mientras se mantiene
  for (const c of vistaTecla.keys()) if (!teclas.has(c)) vistaTecla.delete(c);
  for (const c of teclas) {
    if (t - (vistaTecla.get(c) || 0) > TECLA_MAX_MS) { soltarTodo('tecla sin repetición'); break; }
  }
  const l = leer();
  if (preset) {
    if (l.delanteras || l.traseras || l.inclinar) {
      terminarPreset('cancelado por orden manual');
    } else if (!preset.enganchado) {
      if (!l.on.has('p' + preset.k)) terminarPreset('mantenlo ' + sPreset() + ' s');
      else if (t - preset.t0 >= sPreset() * 1000) Object.assign(preset, {enganchado: true, tVisto: t});
    } else if (t - preset.tVisto > SIN_EVENTOS_MS) {
      terminarPreset('el PC no lo lleva');
    }
  }
  if (modo() !== 'activo') tNeutroHasta = 0;
  else if (hayMando(l)) { tNeutroHasta = t + NEUTRO_MS; enviar(orden(l)); }
  else if (t < tNeutroHasta) enviar(neutro());
  pintarTira(t);
}

/* ── /events ───────────────────────────────────────────────────────────── */
window.estadoRobot.addEventListener('message', ev => {
  let s;
  try { s = JSON.parse(ev.data); } catch (e) { return; }
  const t = performance.now();
  ultimo = s;
  tEvento = t;
  const a = activa();
  if (emer) {
    if (a === 'emergencia') emer.confirmada = true;
    else if (emer.confirmada) emer = null;          // rearmada
  }
  if (rearme && a !== 'emergencia') { rearme = null; avisar('rearmado'); }
  if (hayMando(leer())) {
    if (modo() !== 'activo') soltarTodo('mando desactivado');
    else if (a !== '' && a !== 'web' && a !== 'joy') soltarTodo(MANDA_OTRO[a] || 'manda ' + a);
  }
  seguirPreset(s.mando && s.mando.preset, t);
  aplicarModo();
});

/* ── Pintado: solo textContent y clases, los controles no se regeneran ─── */
function poner(el, texto, clase) {
  if (el.textContent !== texto) el.textContent = texto;
  if (clase !== undefined && el.className !== clase) el.className = clase;
}

function aplicarModo() {
  const md = modo();
  if (md === modoPintado) return;
  modoPintado = md;
  const activo = md === 'activo';
  mando.dataset.modo = md;
  $('mTira').hidden = !activo;
  $('mAbajo').hidden = !activo;
  $('mAyuda').hidden = !activo;
  $('mEmer').hidden = !(activo || md === 'bloqueado');
  $('mLinea').hidden = !(md === 'off' || md === 'bloqueado');
  $('mLinea').textContent = md === 'off'
    ? 'Mando web desactivado (gui_mando:=true)'
    : 'Mando bloqueado: abre el enlace con clave del log';
  document.body.classList.toggle('con-mando', activo);
}

function textoManda(m) {
  const a = m.activa || '', d = m.dueno;
  if (a === 'emergencia') return ['EMERGENCIA WEB', 'e-mal'];
  if (a === 'joy_activo') return ['MANDO FÍSICO', 'e-aviso'];
  if (a === 'web') {
    // dueno.id: el principio de la sesion
    if (d && d.id && sesion.startsWith(d.id)) return ['TÚ', 'e-ok'];
    return ['otro navegador' + (d ? ' (' + d.ip + ')' : ''), 'e-aviso'];
  }
  return [a === 'joy' ? 'nadie (DS4 en reposo)' : 'nadie', ''];
}

function textoRobot(s) {
  if (!s.enlace) return ['sin datos', 'e-mal'];
  if (typeof s.seguridad !== 'boolean') return ['–', ''];
  if (!s.seguridad) return ['obedece', 'e-ok'];
  // bit 6 (watchdog): estado seguro sin ordenes frescas
  return (s.err >> 6) & 1 ? ['en reposo: tracción libre', 'e-aviso'] : ['EMERGENCIA', 'e-mal'];
}

function textoVelocidad(s, m) {
  const n = (v, k, d) => (typeof v === 'number' ? (v * k).toFixed(d) : '–');
  const g = 180 / Math.PI, p = m.pedido || [], o = s.pose || [];
  return `pedida ${n(p[0], 1, 3)} m/s ${n(p[1], g, 1)}°/s · ` +
         `medida ${n(o[3], 1, 3)} m/s ${n(o[4], g, 1)}°/s`;
}

function textoPreset(s, m) {
  const k = preset.k;
  const p = m.preset && m.preset.k === k ? m.preset : null;
  const obj = p && typeof p.objetivo_deg === 'number' ? p.objetivo_deg : PRESETS_DEG[k];
  const brazos = JUNTA.map((j, i) => {
    const q = s.joints ? s.joints[j] : undefined;
    return NOMBRE[i] + ' ' + (typeof q === 'number' ? grados(q * 180 / Math.PI) : '–') +
           ' → ' + grados(obj);
  });
  const fase = preset.enganchado ? (p ? p.estado : 'enganchado') : 'mantén…';
  return 'Preset ' + SIMBOLOS[k] + ' ' + fase + ': ' + brazos.join(' · ');
}

function pintarTira(t) {
  const md = modo();
  if (md !== 'activo' && md !== 'bloqueado') return;
  const s = ultimo || {}, m = s.mando || {}, a = m.activa || '';
  let e = '', ce = 'e-mal';
  if (rearme) { e = 'REARME PEDIDO…'; ce = 'e-aviso'; }
  else if (a === 'emergencia') e = 'ENGANCHADA';
  else if (emer) e = t - emer.t0 < EMER_AVISO_MS ? 'PEDIDA…' : 'NO CONFIRMADA: usa SHARE';
  poner($('mEmerEstado'), e, ce);
  bRearmar.hidden = !(a === 'emergencia' && clave);
  if (md !== 'activo') return;

  poner($('mManda'), ...textoManda(m));
  poner($('mRobot'), ...textoRobot(s));
  const f = frescos(t);
  poner($('mEnlPC'), f ? 'navegador–PC ✓' : 'SIN CONEXIÓN CON EL PC', f ? '' : 'e-mal');
  poner($('mEnlESP'), s.enlace ? 'PC–ESP32 ✓' : 'PC–ESP32 ✕', s.enlace ? '' : 'e-mal');
  $('mSinMux').hidden = m.mux_ok !== false;
  poner($('mVel'), textoVelocidad(s, m));
  $('mPreset').hidden = !preset;
  if (preset) poner($('mPreset'), textoPreset(s, m));
  const ver = !!motivo && t - tMotivo < MOTIVO_MS;
  $('mMotivo').hidden = !ver;
  if (ver) poner($('mMotivo'), motivo);
}

function pintar() {
  const t = performance.now(), l = leer();
  for (const el of controles) el.classList.toggle('on', l.on.has(el.dataset.c));
  // La bola va donde el dedo (o WASD), sin zona muerta
  let x = 0, y = 0;
  for (const p of punteros.values()) if (!p.muerto && p.c === 'pad') { x += p.x; y += p.y; }
  const b = c => (l.on.has(c) ? 1 : 0);
  x += b('d') - b('a'); y += b('w') - b('s');
  const m = Math.max(1, Math.hypot(x, y)), r = (pad.clientWidth - bola.offsetWidth) / 2;
  bola.style.transform = `translate(${x / m * r}px, ${-y / m * r}px)`;
  bPreset.forEach((el, k) => {
    const mio = !!preset && preset.k === k;
    const f = !mio ? 0 : preset.enganchado ? 1 : Math.min(1, (t - preset.t0) / (sPreset() * 1000));
    el.firstElementChild.style.transform = `scaleX(${f})`;
    el.classList.toggle('enganchado', mio && preset.enganchado);
  });
  let fr = rearme ? 1 : 0;
  for (const p of punteros.values()) {
    if (!p.muerto && p.c === 'rearmar') fr = Math.max(fr, Math.min(1, (t - p.t0) / REARME_MS));
  }
  bRearmar.firstElementChild.style.transform = `scaleX(${fr})`;
  requestAnimationFrame(pintar);
}

/* ── Punteros: el estado va por pointerId (dos dedos a la vez) ─────────── */
function posPad(p, e) {
  const r = pad.getBoundingClientRect(), R = r.width / 2;
  let x = (e.clientX - r.left - R) / R, y = (r.top + R - e.clientY) / R;
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  p.x = x; p.y = y;
}

function alPulsar(e) {
  if (e.pointerType === 'mouse' && e.button !== 0) return;
  e.preventDefault();
  const el = e.currentTarget, c = el.dataset.c, t = performance.now();
  if (c === 'emergencia') { emergencia(); return; }
  if (c === 'parar') { soltarTodo('parado'); return; }
  if (c === 'lento') { lento = !lento; el.setAttribute('aria-pressed', String(lento)); return; }
  if (c === 'rearmar') {
    if (activa() !== 'emergencia' || !clave) return;
  } else {
    const por = bloqueo(t);
    if (por) { avisar(por); return; }
  }
  try { el.setPointerCapture(e.pointerId); } catch (err) { /* ya suelto */ }
  const p = {c, t0: t, x: 0, y: 0};
  if (c === 'pad') posPad(p, e);
  punteros.set(e.pointerId, p);
  if (presetDe(c) >= 0) pulsarPreset(presetDe(c));
}

function alMover(e) {
  const p = punteros.get(e.pointerId);
  if (p && !p.muerto && p.c === 'pad') posPad(p, e);
}

function alSoltar(e) { punteros.delete(e.pointerId); }

for (const el of controles) {
  el.addEventListener('pointerdown', alPulsar);
  el.addEventListener('pointermove', alMover);
  el.addEventListener('lostpointercapture', alSoltar);
  el.addEventListener('contextmenu', e => e.preventDefault());
}
// Tambien aqui: si la captura falla, el pointerup llega a otro elemento
window.addEventListener('pointerup', alSoltar, true);
window.addEventListener('pointercancel', alSoltar, true);

/* ── Teclado ───────────────────────────────────────────────────────────── */
document.addEventListener('keydown', e => {
  // Con Cmd, Ctrl o Alt el keyup puede no llegar (macOS): soltar
  if (e.ctrlKey || e.metaKey || e.altKey) {
    if (teclas.size) soltarTodo('tecla modificadora');
    return;
  }
  const c = TECLAS[e.code], md = modo();
  if (!c) return;
  if (md !== 'activo' && !(md === 'bloqueado' && c === 'emergencia')) return;
  e.preventDefault();
  if (e.repeat) { if (teclas.has(c)) vistaTecla.set(c, performance.now()); return; }
  if (c === 'emergencia') { emergencia(); return; }
  if (c === 'parar') { soltarTodo('parado'); return; }
  const por = bloqueo(performance.now());
  if (por) { avisar(por); return; }
  teclas.add(c);
  vistaTecla.set(c, performance.now());
  if (presetDe(c) >= 0) pulsarPreset(presetDe(c));
});

// El menu contextual nativo se queda con el keyup (el lienzo y los mandos
// lo cancelan antes)
window.addEventListener('contextmenu', e => {
  if (!e.defaultPrevented) soltarTodo('menú contextual');
});

document.addEventListener('keyup', e => {
  const c = TECLAS[e.code];
  if (!c) return;
  const tenia = teclas.delete(c), md = modo();
  if (tenia || md === 'activo' || (md === 'bloqueado' && c === 'emergencia')) e.preventDefault();
});

/* ── Soltar todo al perder la pagina ───────────────────────────────────── */
window.addEventListener('blur', () => soltarTodo('ventana sin foco'));
window.addEventListener('pagehide', () => soltarTodo(''));
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') soltarTodo('pestaña oculta');
});
document.addEventListener('freeze', () => soltarTodo(''));
window.addEventListener('hashchange', () => { clave = leerClave(); aplicarModo(); });

new ResizeObserver(() => {
  escena.classList.toggle('estrecho', escena.clientWidth < 640);
}).observe(escena);

setInterval(tick, PERIODO_MS);
setInterval(() => { if (emer && !emer.confirmada) pedirEmergencia(); }, EMER_CADA_MS);
requestAnimationFrame(pintar);
})();
"""

MANDO_HTML = MANDO_HTML.replace(
    '__BOTONES_PRESET__',
    ''.join(_boton_preset(k) for k in range(len(PRESETS_DEG))))

MANDO_JS = (_MANDO_JS
            .replace('__PRESETS_DEG__', json.dumps(list(PRESETS_DEG)))
            .replace('__SIMBOLOS__', json.dumps(list(SIMBOLOS_PRESET),
                                                ensure_ascii=False))
            .replace('__S_PRESET__', json.dumps(S_PRESET)))

PAGINA_3D = (_PAGINA
             .replace('__MANDO_CSS__', MANDO_CSS)
             .replace('__MANDO_HTML__', MANDO_HTML)
             .replace('__MANDO_JS__', MANDO_JS))
