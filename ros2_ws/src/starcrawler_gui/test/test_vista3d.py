"""Tests de la pagina 3D. Corren en el PC, sin ROS ni navegador.

Solo lo que se puede comprobar sin pintar: que las piezas de texto encajan
con el resto del paquete. Lo visual se mira abriendo /3d.
"""
import json
import re
from html.parser import HTMLParser

from starcrawler_common.orugas import INCLINACIONES, PRESETS_DEG

from starcrawler_gui import dashboard, mando_web, mundo_modelo
from starcrawler_gui.vista3d import (
    MANDO_CSS,
    MANDO_HTML,
    MANDO_JS,
    PAGINA_3D,
    SIMBOLOS_PRESET,
    THREE_CDN,
    enlazar_desde_2d,
    pagina_3d,
)

# El cuerpo de POST /mando, en el orden de la especificacion (issue #18)
CAMPOS = ('sesion', 'seq', 'avance', 'giro', 'lento',
          'delanteras', 'traseras', 'inclinar', 'preset')


class _Etiquetas(HTMLParser):
    def __init__(self):
        super().__init__()
        self.etiquetas = []

    def handle_starttag(self, tag, attrs):
        self.etiquetas.append((tag, dict(attrs)))


def _etiquetas(html):
    p = _Etiquetas()
    p.feed(html)
    return p.etiquetas


def _controles():
    return [a for _, a in _etiquetas(MANDO_HTML)
            if 'control' in (a.get('class') or '').split()]


def _modulo():
    return PAGINA_3D[PAGINA_3D.index('<script type="module">'):]


def test_la_url_de_three_se_sustituye():
    html = pagina_3d('/static/three.module.min.js')
    assert "from '/static/three.module.min.js'" in html
    assert '__THREE_URL__' not in html


def test_el_cdn_fija_version():
    """Sin version, un cambio de three.js en el CDN romperia la pagina."""
    assert '@0.' in THREE_CDN and THREE_CDN.endswith('three.module.min.js')


def test_la_pagina_2d_gana_el_enlace_a_la_3d():
    """El ancla es texto de dashboard.py: si cambia alli, esto avisa."""
    html = enlazar_desde_2d(dashboard.PAGINA)
    assert 'href="/3d"' in html
    assert html.count('href="/3d"') == 1


def test_la_3d_lee_del_mismo_stream_que_la_2d():
    """Un solo EventSource: lo abre el mando y la escena se cuelga de el."""
    assert PAGINA_3D.count('EventSource(') == 1
    assert "window.estadoRobot = new EventSource('/events')" in MANDO_JS
    assert "window.estadoRobot.addEventListener('message'" in _modulo()
    assert "fetch('/modelo')" in PAGINA_3D


def test_los_nombres_de_junta_son_los_del_urdf():
    for nombre in ('crawler_fr_joint', 'crawler_fl_joint',
                   'crawler_rr_joint', 'crawler_rl_joint'):
        assert nombre in PAGINA_3D


def test_ninguna_url_externa_nueva():
    """La unica es la de three.js, y se pone al servir la pagina."""
    assert '://' not in PAGINA_3D
    urls = set(re.findall(r'https?://[^\s\'"]+', pagina_3d(THREE_CDN)))
    assert urls == {THREE_CDN}


# ─── Mundo simulado ─────────────────────────────────────────────────────────

def _entre(desde, hasta):
    m = _modulo()
    return m[m.index(desde):m.index(hasta)]


def test_el_mundo_se_pide_cuando_cambia_mundo_v():
    """La geometria nunca va por /events: la pagina la pide a /mundo."""
    m = _modulo()
    assert "fetch('/mundo', {cache: 'no-store'})" in m
    assert 's.mundo_v !== mundoCargado' in m
    assert m.count('fetch(') == 2           # /modelo y /mundo


def test_la_pose_sale_de_verdad_y_si_no_de_pose():
    m = _modulo()
    assert 'const real = s.verdad || s.pose;' in m
    # el mando sigue leyendo la velocidad medida de /odom
    assert 'o = s.pose ||' in MANDO_JS


def test_construir_marca_entiende_los_tipos_de_mundo_modelo():
    trozo = _entre('function construirMarca(', 'function vaciar(')
    for tipo in mundo_modelo.TIPOS.values():
        assert "p.tipo === '%s'" % tipo in trozo, tipo


def test_el_terreno_solo_lee_claves_de_mundo_estado():
    trozo = _entre('function apoya(', 'function poner(')
    usadas = set(re.findall(r'\b[td]\.(\w+)', trozo))
    assert {'estado', 'modo', 'apoya', 'holgura', 'cabeceo'} <= usadas
    assert usadas <= mundo_modelo.CLAVES_ESTADO, usadas - mundo_modelo.CLAVES_ESTADO
    assert 't.z_suelo' in _modulo()


def test_los_estados_son_los_de_mundo_estado():
    bloque = re.search(r'const ESTADOS = \{(.*?)\};', _modulo(), re.S).group(1)
    assert set(re.findall(r'(\w+):', bloque)) == {
        'libre', 'bloqueado', 'sin_traccion', 'cayendo', 'atrapado',
        'volcado'}


def test_los_textos_de_ros_van_con_textcontent():
    """Etiquetas y chip: el texto viene de ROS, nunca como HTML."""
    m = _modulo()
    assert "document.createElement('div')" in m
    assert 'el.textContent = p.texto;' in m
    assert 'chip.textContent = texto;' in m
    # la columna 'apoya' de #orugas solo pinta numeros
    trozo = _entre('function apoya(', 'const ESTADOS')
    assert 'motivo' not in trozo and 'consejo' not in trozo


def test_etiquetas_y_chip_quedan_debajo_del_mando():
    def regla(selector, css):
        cuerpo = re.search(re.escape(selector) + r' \{([^}]*)\}', css).group(1)
        return re.sub(r'\s+', '', cuerpo)
    assert 'z-index:2' in regla('#mando', MANDO_CSS)
    for selector in ('#etiquetas', '#terreno'):
        r = regla(selector, PAGINA_3D)
        assert 'z-index:1' in r and 'pointer-events:none' in r, selector
    # Mismo z-index: manda el orden, y el chip no queda tapado por un rotulo
    ids = [a.get('id') for _, a in _etiquetas(PAGINA_3D)]
    assert ids.index('etiquetas') < ids.index('terreno') < ids.index('mando')


def test_sombras_y_etiquetas_solo_con_mundo():
    botones = {a['id']: a for t, a in _etiquetas(PAGINA_3D)
               if t == 'button' and 'id' in a}
    for nombre in ('bSombras', 'bEtiquetas'):
        assert 'hidden' in botones[nombre]
        assert botones[nombre]['aria-pressed'] == 'true'   # encendidas
    for nombre in ('bPerfil', 'bDetras'):
        assert botones[nombre]['aria-pressed'] == 'false'
    assert 'hayMundo && sombras' in _modulo()


# ─── Mando web ──────────────────────────────────────────────────────────────

def test_el_mando_va_fuera_del_modulo_y_no_nombra_three():
    """Sin three.js el modulo no corre, pero el mando tiene que seguir."""
    assert 'THREE' not in MANDO_JS
    assert '</script' not in MANDO_JS
    i_mando = PAGINA_3D.index(MANDO_JS)
    assert i_mando < PAGINA_3D.index('<script type="module">')
    assert PAGINA_3D[:i_mando].rstrip().endswith('<script>')
    assert PAGINA_3D.index(MANDO_HTML) < PAGINA_3D.index('<aside')
    assert '__' not in PAGINA_3D.replace('__THREE_URL__', '')


def test_los_controles_no_cogen_foco_ni_gestos():
    controles = _controles()
    # pad, 4 pares, 4 de cruceta, presets, parar, lento, rearmar, emergencia
    assert len(controles) == 13 + len(PRESETS_DEG)
    for a in controles:
        assert a.get('tabindex') == '-1', a
    regla = re.search(r'#mando \.control \{([^}]*)\}', MANDO_CSS).group(1)
    regla = re.sub(r'\s+', '', regla)
    for decl in ('touch-action:none', 'user-select:none',
                 '-webkit-touch-callout:none', 'min-width:48px',
                 'min-height:48px'):
        assert decl + ';' in regla, decl


def test_el_mando_no_se_regenera():
    """panel() reescribe #orugas a 10 Hz; nada mas usa innerHTML."""
    assert 'innerHTML' not in MANDO_JS
    assert (PAGINA_3D.count('innerHTML')
            == PAGINA_3D.count("$('orugas').innerHTML"))


def test_cada_control_lo_maneja_el_js():
    for a in _controles():
        assert "'%s'" % a['data-c'] in MANDO_JS, a['data-c']


def test_los_id_no_se_repiten():
    ids = [a['id'] for _, a in _etiquetas(PAGINA_3D) if 'id' in a]
    assert len(ids) == len(set(ids))


def test_el_teclado_es_el_de_la_especificacion():
    bloque = re.search(r'const TECLAS = \{(.*?)\};', MANDO_JS, re.S).group(1)
    teclas = dict(re.findall(r"(\w+): '(\w+)'", bloque))
    assert teclas == {
        'KeyW': 'w', 'KeyA': 'a', 'KeyS': 's', 'KeyD': 'd',
        'ArrowUp': 'arriba', 'ArrowDown': 'abajo',
        'ArrowLeft': 'izq', 'ArrowRight': 'der',
        'KeyR': 'l1', 'KeyF': 'l2', 'KeyT': 'r1', 'KeyG': 'r2',
        'Digit1': 'p0', 'Digit2': 'p1', 'Digit3': 'p2', 'Digit4': 'p3',
        'Space': 'emergencia', 'Escape': 'parar',
    }
    assert 'e.code' in MANDO_JS and 'e.repeat' in MANDO_JS


def test_los_presets_y_la_cruceta_son_los_de_orugas():
    assert len(SIMBOLOS_PRESET) == len(PRESETS_DEG)
    assert json.dumps(list(PRESETS_DEG)) in MANDO_JS
    for k in range(len(PRESETS_DEG)):
        assert 'data-c="p%d"' % k in MANDO_HTML
    for texto in ('✕', '-45°', '○', '0°', '□', '+45°', '△', '+90°'):
        assert texto in MANDO_HTML
    for sentido in INCLINACIONES:
        assert "'%s'" % sentido in MANDO_JS
        assert 'data-c="%s"' % sentido in MANDO_HTML


def test_la_orden_lleva_los_campos_del_servidor():
    """orden() y neutro(): las claves de validar_mando, en el orden del cuerpo."""
    assert set(CAMPOS) == mando_web.CLAVES
    assert 'Object.assign({sesion, seq: seq++}, o)' in MANDO_JS
    literales = re.findall(r'\{avance: [^}]*\}', MANDO_JS)
    assert len(literales) == 2
    for literal in literales:
        claves = re.findall(r'(?:^|[{,]\s*)(\w+)(?=\s*[:,}])', literal)
        assert tuple(['sesion', 'seq'] + claves) == CAMPOS


def test_la_sesion_cumple_el_formato():
    """Hexadecimal, dentro de lo que acepta validar_mando."""
    n = int(re.search(r'new Uint8Array\((\d+)\)', MANDO_JS).group(1))
    assert mando_web._SESION.fullmatch('a' * (2 * n))
    assert 'crypto.getRandomValues' in MANDO_JS


def test_la_clave_va_en_la_cabecera_y_sale_de_la_url():
    assert "'%s'" % mando_web.CABECERA_CLAVE in MANDO_JS
    assert "history.replaceState(null, '', '/3d')" in MANDO_JS
    assert 'sessionStorage' in MANDO_JS
    assert '?t=' not in MANDO_JS


def test_las_rutas_son_las_del_servidor():
    for ruta in mando_web.RUTAS_POST:
        assert "fetch('%s'" % ruta in MANDO_JS


def test_los_finales_de_preset_son_los_del_arbitro():
    """Un estado vivo tomado por final soltaria el preset antes de tiempo."""
    terminales = re.search(r'const TERMINALES = \[([^\]]*)\]', MANDO_JS).group(1)
    terminales = set(re.findall(r"'([^']*)'", terminales))
    assert terminales == {mando_web.LLEGADO, mando_web.CANCELADO,
                          mando_web.SIN_ENCODER, mando_web.SIN_ENLACE,
                          mando_web.TIEMPO}
    assert not terminales & set(mando_web._VIVOS)
