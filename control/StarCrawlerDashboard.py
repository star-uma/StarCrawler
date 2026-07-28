#!/usr/bin/env python3
"""
StarCrawlerDashboard.py
=======================
Dashboard web en tiempo real de StarCrawler: dibuja el robot (las 4 orugas
con su angulo real), velocidades de traccion, modo activo, errores y
graficas de historico. Corre en el PC y se ve en el navegador.

Fuentes de telemetria (elegir una):

  python StarCrawlerDashboard.py --serial COM7   # standalone por USB (lineas TLM)
  python StarCrawlerDashboard.py --udp           # firmwares WiFi (puerto 8886)
  python StarCrawlerDashboard.py --demo          # datos sinteticos, sin robot

Luego abrir http://localhost:8000 (se abre solo).

Sin dependencias salvo pyserial para --serial (pip install pyserial).
Nota: --udp usa el puerto 8886, el mismo que los tests HIL: no lanzar ambos.
"""
import argparse
import json
import math
import socket
import struct
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ─── Estado compartido ────────────────────────────────────────────────────────

LOCK = threading.Lock()
ESTADO = {
    "fuente": "-",
    "modo": 0,
    "vl": None, "vr": None,          # dps (solo serie/demo)
    "ang": [180.0, 180.0, 180.0, 180.0],  # {FR, FL, RR, RL}
    "roll": None, "pitch": None,     # solo UDP (version completa)
    "err": 0,
    "con_imu": False,                # la fuente aporta roll/pitch
    "ts": 0.0,
}
EVENTOS = deque(maxlen=10)   # lineas [MANDO]/[MODO]/[SEGURIDAD] del serie
PAQUETES = deque(maxlen=100)  # timestamps para calcular paquetes/s


def registrar(**campos):
    with LOCK:
        ESTADO.update(campos)
        ESTADO["ts"] = time.monotonic()
        PAQUETES.append(ESTADO["ts"])


def instantanea():
    with LOCK:
        ahora = time.monotonic()
        recientes = [t for t in PAQUETES if ahora - t < 2.0]
        s = dict(ESTADO)
        s["pps"] = round(len(recientes) / 2.0, 1)
        s["enlace"] = (ahora - ESTADO["ts"]) < 1.0 if ESTADO["ts"] else False
        s["eventos"] = list(EVENTOS)
    return s


# ─── Fuentes de telemetria ────────────────────────────────────────────────────

def fuente_udp(puerto):
    """Telemetria binaria de los firmwares WiFi: 9 x int16 LE, id=2."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", puerto))
    sock.settimeout(0.5)
    print(f"[UDP] Escuchando telemetria en puerto {puerto}...")
    while True:
        try:
            datos, _ = sock.recvfrom(64)
        except socket.timeout:
            continue
        if len(datos) < 18:
            continue
        c = struct.unpack("<9h", datos[:18])
        if c[0] != 2:
            continue
        registrar(
            fuente="udp",
            ang=[c[1] / 100.0, c[2] / 100.0, c[3] / 100.0, c[4] / 100.0],
            roll=c[5] / 100.0, pitch=c[6] / 100.0,
            modo=c[7], err=c[8] & 0xFFFF,
            vl=None, vr=None, con_imu=True,
        )


def fuente_serie(puerto):
    """Lineas TLM del firmware standalone por USB (115200 baudios)."""
    try:
        import serial
    except ImportError:
        sys.exit("Falta pyserial para --serial:  pip install pyserial")
    s = serial.Serial(puerto, 115200, timeout=0.5)
    print(f"[SERIE] Leyendo {puerto} a 115200...")
    while True:
        try:
            linea = s.readline().decode("utf-8", errors="replace").strip()
        except Exception as e:
            print(f"[SERIE] Error: {e}. Reintentando en 2 s...")
            time.sleep(2)
            continue
        if not linea:
            continue
        if linea.startswith("TLM,"):
            partes = linea.split(",")
            if len(partes) != 9:
                continue
            try:
                registrar(
                    fuente="serie",
                    modo=int(partes[1]),
                    vl=float(partes[2]), vr=float(partes[3]),
                    ang=[float(partes[4]), float(partes[5]),
                         float(partes[6]), float(partes[7])],
                    err=int(partes[8]),
                    roll=None, pitch=None, con_imu=False,
                )
            except ValueError:
                continue
        elif linea.startswith("["):
            with LOCK:
                EVENTOS.append(time.strftime("%H:%M:%S ") + linea)
            print(linea)


def fuente_demo():
    """Datos sinteticos para probar el dashboard sin robot."""
    print("[DEMO] Generando telemetria sintetica a 10 Hz...")
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        modo = [1, 1, 2, 3, 4][int(t / 6) % 5]
        ang = [180 + 60 * math.sin(t * 0.7 + i * 1.3) for i in range(4)]
        vl = 30 * math.sin(t * 0.9) if modo == 1 else 0.0
        vr = 30 * math.sin(t * 0.9 + 0.4) if modo == 1 else 0.0
        err = 0b0000100 if int(t) % 20 > 16 else 0  # RR falla a ratos
        registrar(fuente="demo", modo=modo, vl=round(vl, 1), vr=round(vr, 1),
                  ang=[round(a, 1) for a in ang], err=err,
                  roll=None, pitch=None, con_imu=False)
        time.sleep(0.1)


# ─── Servidor HTTP + SSE ──────────────────────────────────────────────────────

class Manejador(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            cuerpo = PAGINA.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.end_headers()
            self.wfile.write(cuerpo)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    datos = json.dumps(instantanea())
                    self.wfile.write(f"data: {datos}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return
        else:
            self.send_error(404)


# ─── Pagina (HTML + JS embebidos, sin dependencias externas) ─────────────────

PAGINA = r"""<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<title>StarCrawler — telemetria</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  /* Paleta validada (modo oscuro): consola de control */
  :root {
    --pagina:#0d0d0d; --superficie:#1a1a19; --borde:rgba(255,255,255,.10);
    --tinta:#ffffff; --tinta2:#c3c2b7; --apagado:#898781;
    --rejilla:#2c2c2a; --eje:#383835;
    --s1:#3987e5; --s2:#008300; --s3:#d55181; --s4:#c98500; /* FR FL RR RL */
    --s5:#199e70; --s6:#d95926;                               /* vel izq/der */
    --ok:#0ca30c; --critico:#d03b3b; --aviso:#fab219;
  }
  * { box-sizing:border-box; margin:0; }
  body { background:var(--pagina); color:var(--tinta);
         font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; padding:14px; }
  .fila { display:flex; gap:12px; flex-wrap:wrap; margin-bottom:12px; }
  .tarjeta { background:var(--superficie); border:1px solid var(--borde);
             border-radius:10px; padding:12px 14px; }
  h1 { font-size:16px; letter-spacing:.06em; }
  h2 { font-size:11px; color:var(--apagado); text-transform:uppercase;
       letter-spacing:.1em; margin-bottom:8px; font-weight:600; }
  .cab { display:flex; align-items:center; gap:14px; width:100%; }
  .punto { width:10px; height:10px; border-radius:50%; background:var(--critico); }
  .punto.on { background:var(--ok); }
  .sub { color:var(--tinta2); font-size:12px; }
  #modo { font-size:34px; font-weight:700; }
  #modoNombre { color:var(--tinta2); }
  .chips { display:grid; grid-template-columns:repeat(2,1fr); gap:6px; }
  .chip { display:flex; gap:6px; align-items:center; font-size:12px;
          color:var(--tinta2); padding:3px 8px; border-radius:6px;
          border:1px solid var(--borde); }
  .chip b { font-weight:600; }
  .chip.mal { border-color:var(--critico); color:var(--tinta); }
  canvas { display:block; }
  .leyenda { display:flex; gap:14px; font-size:12px; color:var(--tinta2);
             margin-bottom:6px; flex-wrap:wrap; }
  .leyenda i { display:inline-block; width:10px; height:10px; border-radius:2px;
               margin-right:5px; vertical-align:-1px; }
  .num { font-variant-numeric:tabular-nums; }
  #eventos { font:11px/1.6 ui-monospace,Consolas,monospace; color:var(--tinta2);
             white-space:pre-wrap; }
  .barra { position:relative; height:22px; background:var(--rejilla);
           border-radius:5px; overflow:hidden; margin:5px 0 10px; }
  .barra > div { position:absolute; top:0; bottom:0; background:var(--s1); }
  .cero { position:absolute; left:50%; top:0; bottom:0; width:1px;
          background:var(--eje); }
  .vlabel { display:flex; justify-content:space-between; font-size:12px;
            color:var(--tinta2); }
</style></head><body>

<div class="fila tarjeta cab">
  <h1>STARCRAWLER · TELEMETRÍA</h1>
  <span class="punto" id="enlace"></span>
  <span class="sub" id="info">esperando datos…</span>
</div>

<div class="fila">
  <div class="tarjeta" style="width:170px">
    <h2>Modo activo</h2>
    <div id="modo" class="num">–</div>
    <div id="modoNombre">sin datos</div>
  </div>
  <div class="tarjeta" style="flex:1;min-width:560px">
    <h2>Robot en vivo — vista lateral (ángulo de cada oruga)</h2>
    <canvas id="robot" width="820" height="240"></canvas>
  </div>
  <div class="tarjeta" style="width:230px">
    <h2>Estado</h2>
    <div class="chips" id="chips"></div>
    <h2 style="margin-top:12px">Tracción (dps)</h2>
    <div class="vlabel"><span>Izq</span><b class="num" id="vlTxt">–</b></div>
    <div class="barra"><div id="vlBar"></div><div class="cero"></div></div>
    <div class="vlabel"><span>Der</span><b class="num" id="vrTxt">–</b></div>
    <div class="barra"><div id="vrBar"></div><div class="cero"></div></div>
  </div>
</div>

<div class="fila">
  <div class="tarjeta" style="flex:2;min-width:520px">
    <h2>Ángulos de oruga — últimos 60 s</h2>
    <div class="leyenda">
      <span><i style="background:var(--s1)"></i>FR</span>
      <span><i style="background:var(--s2)"></i>FL</span>
      <span><i style="background:var(--s3)"></i>RR</span>
      <span><i style="background:var(--s4)"></i>RL</span>
    </div>
    <canvas id="gAng" width="760" height="170"></canvas>
  </div>
  <div class="tarjeta" style="flex:1;min-width:340px">
    <h2>Velocidades — últimos 60 s</h2>
    <div class="leyenda">
      <span><i style="background:var(--s5)"></i>Izq</span>
      <span><i style="background:var(--s6)"></i>Der</span>
    </div>
    <canvas id="gVel" width="420" height="150"></canvas>
    <h2 style="margin-top:10px">Eventos</h2>
    <div id="eventos">—</div>
  </div>
</div>

<script>
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const COLOR = [css('--s1'), css('--s2'), css('--s3'), css('--s4')];
const NOMBRE = ['FR','FL','RR','RL'];
const MODOS = {0:'seguridad / parado',1:'tracción',2:'posición absoluta',
               3:'incremental ×4',4:'incremental ×2',5:'nivelado automático',
               6:'simultáneo'};
const HIST = [];                  // {t, ang[4], vl, vr}
const VENTANA = 60;               // segundos

let ultimo = null;

const es = new EventSource('/events');
es.onmessage = ev => {
  const s = JSON.parse(ev.data);
  ultimo = s;
  const t = performance.now() / 1000;
  if (s.enlace) {
    HIST.push({t, ang: s.ang.slice(), vl: s.vl, vr: s.vr});
    while (HIST.length && t - HIST[0].t > VENTANA) HIST.shift();
  }
  pintar(s, t);
};

function pintar(s, t) {
  // Cabecera
  document.getElementById('enlace').className = 'punto' + (s.enlace ? ' on' : '');
  document.getElementById('info').textContent =
    `fuente: ${s.fuente} · ${s.pps} paq/s` + (s.enlace ? '' : ' · SIN ENLACE');

  // Modo
  document.getElementById('modo').textContent = s.enlace ? s.modo : '–';
  document.getElementById('modoNombre').textContent =
    s.enlace ? (MODOS[s.modo] || '?') : 'sin datos';

  // Chips de estado (icono + etiqueta: nunca solo color)
  const bits = [[0,'ENC FR'],[1,'ENC FL'],[2,'ENC RR'],[3,'ENC RL']];
  if (s.con_imu) bits.push([4,'IMU']);
  bits.push([5,'CAN'],[6,'WATCHDOG']);
  document.getElementById('chips').innerHTML = bits.map(([b, nom]) => {
    const mal = (s.err >> b) & 1;
    return `<span class="chip${mal ? ' mal' : ''}">${mal ? '✕' : '✓'} <b>${nom}</b></span>`;
  }).join('');

  // Barras de velocidad (solo fuentes con vl/vr)
  ponBarra('vl', s.vl); ponBarra('vr', s.vr);

  robot(s);
  grafica(document.getElementById('gAng'),
          [0,1,2,3].map(i => ({c: COLOR[i], f: p => p.ang[i]})), 60, 300, [90,180,270]);
  grafica(document.getElementById('gVel'),
          [{c: css('--s5'), f: p => p.vl}, {c: css('--s6'), f: p => p.vr}],
          -45, 45, [0]);

  document.getElementById('eventos').textContent =
    (s.eventos && s.eventos.length) ? s.eventos.slice(-8).join('\n') : '—';
}

function ponBarra(id, v) {
  const bar = document.getElementById(id + 'Bar');
  const txt = document.getElementById(id + 'Txt');
  if (v === null || v === undefined) { txt.textContent = '–'; bar.style.width = 0; return; }
  txt.textContent = v.toFixed(1);
  const frac = Math.max(-1, Math.min(1, v / 40));
  bar.style.left = frac < 0 ? (50 + frac * 50) + '%' : '50%';
  bar.style.width = Math.abs(frac) * 50 + '%';
}

/* ── Dibujo del robot: dos vistas laterales, brazos con su ángulo real ──── */
function robot(s) {
  const cv = document.getElementById('robot'), g = cv.getContext('2d');
  g.clearRect(0, 0, cv.width, cv.height);
  // vista izquierda: FL(1) delante, RL(3) detrás · vista derecha: FR(0), RR(2)
  vista(g,  10, 'LADO IZQUIERDO', s, 1, 3);
  vista(g, 420, 'LADO DERECHO',   s, 0, 2);
}

function vista(g, x0, titulo, s, iDel, iTra) {
  const W = 390, cy = 120, cuerpoW = 150, brazo = 95;
  const cx = x0 + W / 2;
  g.font = '10px system-ui'; g.fillStyle = css('--apagado');
  g.fillText(titulo + '   (frente →)', x0 + 8, 16);
  // suelo
  g.strokeStyle = css('--eje'); g.lineWidth = 1;
  g.beginPath(); g.moveTo(x0, 225); g.lineTo(x0 + W, 225); g.stroke();
  // chasis
  g.fillStyle = css('--rejilla'); g.strokeStyle = css('--tinta2'); g.lineWidth = 1.5;
  caja(g, cx - cuerpoW / 2, cy - 22, cuerpoW, 44, 8); g.fill(); g.stroke();
  // pivotes: delantero a la derecha, trasero a la izquierda
  const pivotes = [[cx + cuerpoW / 2, cy, +1, iDel], [cx - cuerpoW / 2, cy, -1, iTra]];
  for (const [px, py, dir, i] of pivotes) {
    const okEnc = !((s.err >> i) & 1);
    // elevacion visual: FR/RL espejan respecto a FL/RR (convencion del TFG)
    const signo = (i === 0 || i === 3) ? 1 : -1;
    const elev = s.enlace ? signo * (180 - s.ang[i]) : 0;
    const rad = elev * Math.PI / 180;
    const tx = px + dir * brazo * Math.cos(rad), ty = py - brazo * Math.sin(rad);
    // oruga (trazo grueso redondeado)
    g.lineCap = 'round'; g.lineWidth = 16;
    g.strokeStyle = okEnc ? COLOR[i] : css('--rejilla');
    if (!okEnc) g.setLineDash([6, 6]);
    g.beginPath(); g.moveTo(px, py); g.lineTo(tx, ty); g.stroke();
    g.setLineDash([]);
    // linea interior + rueda
    g.lineWidth = 2; g.strokeStyle = css('--pagina');
    g.beginPath(); g.moveTo(px, py); g.lineTo(tx, ty); g.stroke();
    g.fillStyle = css('--tinta2');
    g.beginPath(); g.arc(px, py, 5, 0, 7); g.fill();
    // etiqueta: nombre + angulo (texto en tinta, no en color de serie),
    // acotada a los bordes del panel para que nunca se corte
    g.font = '11px system-ui'; g.fillStyle = css('--tinta');
    const et = okEnc && s.enlace ? `${NOMBRE[i]} ${s.ang[i].toFixed(1)}°`
                                 : `${NOMBRE[i]} ✕ ENC`;
    const lx = Math.max(x0 + 4, Math.min(tx - 24, x0 + W - 68));
    const ly = Math.max(30, Math.min(ty - 14, 218));
    g.fillText(et, lx, ly);
  }
}

function caja(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y); g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r); g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r); g.closePath();
}

/* ── Grafica de lineas generica (2 px, rejilla tenue, sin doble eje) ────── */
function grafica(cv, series, yMin, yMax, refs) {
  const g = cv.getContext('2d'), W = cv.width, H = cv.height, mIzq = 34;
  g.clearRect(0, 0, W, H);
  const ahora = performance.now() / 1000;
  const X = t => mIzq + (W - mIzq - 6) * (1 - (ahora - t) / VENTANA);
  const Y = v => H - 16 - (H - 26) * (v - yMin) / (yMax - yMin);
  // rejilla y referencias
  g.font = '10px system-ui';
  for (const r of refs) {
    g.strokeStyle = css('--rejilla'); g.lineWidth = 1;
    g.beginPath(); g.moveTo(mIzq, Y(r)); g.lineTo(W - 6, Y(r)); g.stroke();
    g.fillStyle = css('--apagado'); g.fillText(r, 4, Y(r) + 3);
  }
  for (const srs of series) {
    g.strokeStyle = srs.c; g.lineWidth = 2; g.beginPath();
    let primero = true;
    for (const p of HIST) {
      const v = srs.f(p);
      if (v === null || v === undefined) continue;
      const x = X(p.t), y = Y(Math.max(yMin, Math.min(yMax, v)));
      primero ? g.moveTo(x, y) : g.lineTo(x, y);
      primero = false;
    }
    g.stroke();
  }
}
</script></body></html>
"""


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Dashboard StarCrawler")
    grupo = ap.add_mutually_exclusive_group()
    grupo.add_argument("--serial", metavar="COMx",
                       help="leer telemetria TLM del firmware standalone")
    grupo.add_argument("--udp", action="store_true",
                       help="escuchar telemetria UDP (firmwares WiFi)")
    grupo.add_argument("--demo", action="store_true",
                       help="datos sinteticos para probar sin robot")
    ap.add_argument("--http", type=int, default=8000, metavar="PUERTO",
                    help="puerto web local (defecto 8000)")
    ap.add_argument("--udp-port", type=int, default=8886,
                    help="puerto de telemetria UDP (defecto 8886)")
    ap.add_argument("--no-abrir", action="store_true",
                    help="no abrir el navegador automaticamente")
    args = ap.parse_args()

    if args.serial:
        objetivo, arg = fuente_serie, (args.serial,)
    elif args.demo:
        objetivo, arg = fuente_demo, ()
    else:
        objetivo, arg = fuente_udp, (args.udp_port,)

    threading.Thread(target=objetivo, args=arg, daemon=True).start()

    servidor = ThreadingHTTPServer(("", args.http), Manejador)
    url = f"http://localhost:{args.http}"
    print(f"[WEB] Dashboard en {url}  (Ctrl+C para salir)")
    if not args.no_abrir:
        webbrowser.open(url)
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        print("\n[WEB] Cerrado.")


if __name__ == "__main__":
    main()
