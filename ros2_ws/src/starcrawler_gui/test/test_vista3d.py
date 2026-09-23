"""Tests de la pagina 3D. Corren en el PC, sin ROS ni navegador.

Solo lo que se puede comprobar sin pintar: que las piezas de texto encajan
con el resto del paquete. Lo visual se mira abriendo /3d.
"""
from starcrawler_gui import dashboard
from starcrawler_gui.vista3d import (
    PAGINA_3D,
    THREE_CDN,
    enlazar_desde_2d,
    pagina_3d,
)


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
    assert "new EventSource('/events')" in PAGINA_3D
    assert "fetch('/modelo')" in PAGINA_3D


def test_los_nombres_de_junta_son_los_del_urdf():
    for nombre in ('crawler_fr_joint', 'crawler_fl_joint',
                   'crawler_rr_joint', 'crawler_rl_joint'):
        assert nombre in PAGINA_3D
