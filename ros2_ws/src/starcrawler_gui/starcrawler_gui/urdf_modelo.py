"""
urdf_modelo.py — el URDF traducido a algo que pueda dibujar el navegador
========================================================================
Modulo PURO: no importa rclpy. Recibe el XML ya expandido (el que publica
robot_state_publisher en /robot_description) y devuelve un diccionario que
se sirve tal cual como JSON a la vista 3D.

Asi la web dibuja exactamente el mismo modelo que RViz: si cambian las
cotas del xacro, cambian en los dos sitios a la vez.

Solo entiende primitivas (box, cylinder, sphere). Las mallas se cuentan en
`mallas_omitidas` pero no se dibujan.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Optional

GRIS = [0.6, 0.6, 0.6, 1.0]


def _floats(texto: Optional[str], por_defecto: List[float]) -> List[float]:
    if not texto:
        return list(por_defecto)
    return [float(v) for v in texto.split()]


def _origen(elem: Optional[ET.Element]) -> Dict[str, List[float]]:
    if elem is None:
        return {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]}
    return {'xyz': _floats(elem.get('xyz'), [0.0, 0.0, 0.0]),
            'rpy': _floats(elem.get('rpy'), [0.0, 0.0, 0.0])}


def _color(material: Optional[ET.Element],
           materiales: Dict[str, List[float]]) -> List[float]:
    if material is None:
        return list(GRIS)
    color = material.find('color')
    if color is not None:
        return _floats(color.get('rgba'), GRIS)
    return list(materiales.get(material.get('name', ''), GRIS))


def _geometria(geom: ET.Element) -> Optional[Dict]:
    caja = geom.find('box')
    if caja is not None:
        return {'tipo': 'caja', 'tam': _floats(caja.get('size'), [0, 0, 0])}
    cil = geom.find('cylinder')
    if cil is not None:
        return {'tipo': 'cilindro', 'radio': float(cil.get('radius', 0)),
                'largo': float(cil.get('length', 0))}
    esf = geom.find('sphere')
    if esf is not None:
        return {'tipo': 'esfera', 'radio': float(esf.get('radius', 0))}
    return None


def leer_urdf(xml: str) -> Dict:
    """URDF expandido -> {raiz, links, joints, mallas_omitidas}."""
    robot = ET.fromstring(xml)

    # Materiales con nombre a nivel de robot: las visuales los referencian
    materiales = {}
    for m in robot.findall('material'):
        color = m.find('color')
        if color is not None:
            materiales[m.get('name')] = _floats(color.get('rgba'), GRIS)

    links = {}
    omitidas = 0
    for link in robot.findall('link'):
        visuales = []
        for vis in link.findall('visual'):
            geom = vis.find('geometry')
            forma = _geometria(geom) if geom is not None else None
            if forma is None:
                omitidas += 1
                continue
            forma['origen'] = _origen(vis.find('origin'))
            forma['color'] = _color(vis.find('material'), materiales)
            visuales.append(forma)
        links[link.get('name')] = visuales

    joints = []
    hijos = set()
    for j in robot.findall('joint'):
        padre = j.find('parent').get('link')
        hijo = j.find('child').get('link')
        hijos.add(hijo)
        eje = j.find('axis')
        joints.append({
            'nombre': j.get('name'),
            'tipo': j.get('type'),
            'padre': padre,
            'hijo': hijo,
            'origen': _origen(j.find('origin')),
            # URDF: sin <axis> el eje es X
            'eje': _floats(eje.get('xyz') if eje is not None else None,
                           [1.0, 0.0, 0.0]),
        })

    raices = [n for n in links if n not in hijos]
    if len(raices) != 1:
        raise ValueError('El URDF debe tener una sola raiz, tiene %d: %s'
                         % (len(raices), raices))

    return {'raiz': raices[0], 'links': links, 'joints': joints,
            'mallas_omitidas': omitidas}
