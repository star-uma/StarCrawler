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

three.js se carga de una copia local si existe (static/, ver README del
paquete) y si no del CDN. Sin internet y sin copia, la pagina lo avisa.
"""

THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js'

# Donde se inserta el enlace a esta vista en la pagina 2D de dashboard.py
_ANCLA_2D = '<span class="sub" id="info">esperando datos…</span>'
_ENLACE_2D = ('<a href="/3d" style="margin-left:auto;color:var(--s1);'
              'text-decoration:none">Vista 3D →</a>')


def pagina_3d(url_three: str) -> str:
    return PAGINA_3D.replace('__THREE_URL__', url_three)


def enlazar_desde_2d(pagina: str) -> str:
    """Anade el enlace a /3d en la cabecera de la pagina 2D."""
    return pagina.replace(_ANCLA_2D, _ANCLA_2D + _ENLACE_2D, 1)


PAGINA_3D = r"""<!DOCTYPE html>
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
  .oruga { display:grid; grid-template-columns:12px 28px 1fr auto; gap:8px;
           align-items:center; font-size:13px; padding:2px 0; }
  .oruga i { width:12px; height:12px; border-radius:3px; }
  .oruga .mal { color:var(--critico); }
  .botones { display:flex; flex-wrap:wrap; gap:6px; }
  button { font:inherit; font-size:12px; color:var(--tinta);
           background:var(--rejilla); border:1px solid var(--borde);
           border-radius:6px; padding:5px 10px; cursor:pointer; }
  button[aria-pressed="true"] { border-color:var(--s1); color:var(--s1); }
  .ayuda { font-size:11px; color:var(--apagado); margin-top:8px; }
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
    <div id="aviso">Cargando la vista…</div>
  </div>
  <aside class="tarjeta">
    <h2>Posición (odom)</h2>
    <div class="dato"><span>x</span><b class="num" id="px">–</b></div>
    <div class="dato"><span>y</span><b class="num" id="py">–</b></div>
    <div class="dato"><span>rumbo</span><b class="num" id="pyaw">–</b></div>
    <div class="dato"><span>recorrido</span><b class="num" id="pdist">–</b></div>
    <h2>Velocidad</h2>
    <div class="dato"><span>lineal</span><b class="num" id="vlin">–</b></div>
    <div class="dato"><span>giro</span><b class="num" id="vang">–</b></div>
    <h2>Orugas (elevación)</h2>
    <div id="orugas"></div>
    <h2>Cámara</h2>
    <div class="botones">
      <button id="bSeguir" aria-pressed="true">Seguir al robot</button>
      <button id="bCenital">Cenital</button>
      <button id="bRastro">Borrar rastro</button>
    </div>
    <p class="ayuda">Arrastrar: girar · Rueda o pellizco: zoom ·
      Mayús + arrastrar o botón derecho: desplazar</p>
  </aside>
</main>

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
contenedor.prepend(renderer.domElement);

const escena = new THREE.Scene();
const camara = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
camara.up.set(0, 0, 1);

escena.add(new THREE.HemisphereLight(0xffffff, 0x303030, 2.2));
const sol = new THREE.DirectionalLight(0xffffff, 1.8);
sol.position.set(3, -2, 6);
escena.add(sol);

// Suelo de 20 x 20 m en celdas de 0,5 m, como plano.rviz
const rejilla = new THREE.GridHelper(20, 40, css('--eje'), css('--rejilla'));
rejilla.rotation.x = Math.PI / 2;
escena.add(rejilla);
escena.add(new THREE.AxesHelper(0.5));   // origen de odom: X rojo, Y verde

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
    juntas[j.nombre] = {tipo: j.tipo, movil, hijo: links[j.hijo],
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

/* ── Rastro del recorrido ─────────────────────────────────────────────── */
const MAX_RASTRO = 20000;
const puntos = new Float32Array(MAX_RASTRO * 3);
const geoRastro = new THREE.BufferGeometry();
geoRastro.setAttribute('position', new THREE.BufferAttribute(puntos, 3));
geoRastro.setDrawRange(0, 0);
const rastro = new THREE.Line(geoRastro, new THREE.LineBasicMaterial({color: css('--s1')}));
rastro.frustumCulled = false;   // la esfera envolvente no se recalcula
escena.add(rastro);
let nRastro = 0;

function anadirRastro(x, y) {
  if (nRastro > 0) {
    const i = (nRastro - 1) * 3;
    if (Math.hypot(x - puntos[i], y - puntos[i + 1]) < 0.02) return;
  }
  if (nRastro >= MAX_RASTRO) {             // lleno: se tira la mitad antigua
    puntos.copyWithin(0, (MAX_RASTRO / 2) * 3);
    nRastro = MAX_RASTRO / 2;
  }
  puntos.set([x, y, 0.004], nRastro * 3);
  nRastro++;
  geoRastro.setDrawRange(0, nRastro);
  geoRastro.attributes.position.needsUpdate = true;
}

function borrarRastro() { nRastro = 0; geoRastro.setDrawRange(0, 0); }

/* ── Datos: el mismo /events que la vista 2D ──────────────────────────── */
// Llegan a 10 Hz; se suaviza hacia el ultimo valor para que no de saltos
const objetivo = {x: 0, y: 0, yaw: 0, juntas: {}};
const visto = {x: 0, y: 0, yaw: 0, juntas: {}};
let hayPose = false, recorrido = 0;

const difAngulo = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));

new EventSource('/events').onmessage = ev => {
  const s = JSON.parse(ev.data);
  if (s.pose) {
    const [x, y, yaw] = s.pose;
    const salto = Math.hypot(x - objetivo.x, y - objetivo.y);
    if (!hayPose || salto > 1.0) {
      // Primera pose, o la odometria se ha reiniciado: sin transicion
      Object.assign(visto, {x, y, yaw});
      if (hayPose) { borrarRastro(); recorrido = 0; }
    } else {
      recorrido += salto;
    }
    Object.assign(objetivo, {x, y, yaw});
    hayPose = true;
    // Aqui y no en el bucle de dibujo: el navegador frena ese bucle con la
    // pestana en segundo plano y el rastro saldria a trozos rectos
    anadirRastro(x, y);
  }
  if (s.joints) Object.assign(objetivo.juntas, s.joints);
  panel(s);
};

function fmt(v, dec, unidad) {
  return (v === null || v === undefined) ? '–' : v.toFixed(dec) + ' ' + unidad;
}

function panel(s) {
  $('enlace').className = 'punto' + (s.enlace ? ' on' : '');
  $('info').textContent = `fuente: ${s.fuente} · ${s.pps} paq/s` +
    (s.enlace ? '' : ' · SIN ENLACE');
  $('sinEnlace').hidden = !!s.enlace;

  const p = s.pose;
  $('px').textContent = p ? fmt(p[0], 2, 'm') : 'sin /odom';
  $('py').textContent = p ? fmt(p[1], 2, 'm') : '–';
  $('pyaw').textContent = p ? fmt(p[2] * 180 / Math.PI, 0, '°') : '–';
  $('pdist').textContent = p ? fmt(recorrido, 2, 'm') : '–';
  $('vlin').textContent = p ? fmt(p[3], 2, 'm/s') : '–';
  $('vang').textContent = p ? fmt(p[4] * 180 / Math.PI, 1, '°/s') : '–';

  $('orugas').innerHTML = JUNTA.map((nombre, i) => {
    const q = s.joints ? s.joints[nombre] : undefined;
    const ok = !((s.err >> i) & 1);
    const ang = (q === undefined) ? '–' : (q >= 0 ? '+' : '') + (q * 180 / Math.PI).toFixed(1) + '°';
    return `<div class="oruga"><i style="background:${COLOR[i]}"></i><b>${NOMBRE[i]}</b>` +
      `<span class="num">${ang}</span>` +
      `<span class="${ok ? '' : 'mal'}">${ok ? '✓' : '✕ enc'}</span></div>`;
  }).join('');

  if (robot) {
    JUNTA.forEach((nombre, i) => {
      if (robot.juntas[nombre]) marcarEncoder(robot.juntas[nombre], !((s.err >> i) & 1));
    });
  }
}

/* ── Camara orbital ───────────────────────────────────────────────────── */
const orbita = {az: -2.3, el: 0.55, dist: 4, centro: new THREE.Vector3(0, 0, 0.1),
                seguir: true};
const punteros = new Map();
let pellizco = 0;

function colocarCamara() {
  if (orbita.seguir) orbita.centro.set(visto.x, visto.y, 0.1);
  const c = Math.cos(orbita.el);
  camara.position.set(
    orbita.centro.x + orbita.dist * c * Math.cos(orbita.az),
    orbita.centro.y + orbita.dist * c * Math.sin(orbita.az),
    orbita.centro.z + orbita.dist * Math.sin(orbita.el));
  camara.lookAt(orbita.centro);
}

function seguir(si) {
  orbita.seguir = si;
  $('bSeguir').setAttribute('aria-pressed', String(si));
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
$('bCenital').onclick = () => { orbita.el = 1.55; orbita.az = -Math.PI / 2; };
$('bRastro').onclick = () => { borrarRastro(); recorrido = 0; };

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
  for (const [n, q] of Object.entries(objetivo.juntas)) {
    const antes = visto.juntas[n] ?? q;
    visto.juntas[n] = antes + (q - antes) * k;
  }

  if (robot) {
    robot.raiz.position.set(visto.x, visto.y, 0);
    robot.raiz.rotation.set(0, 0, visto.yaw);
    for (const [n, q] of Object.entries(visto.juntas)) {
      if (robot.juntas[n]) moverJunta(robot.juntas[n], q);
    }
  }
  colocarCamara();
  renderer.render(escena, camara);
  requestAnimationFrame(bucle);
}

aviso('');
cargarModelo();
requestAnimationFrame(bucle);
</script></body></html>
"""
