"""El mundo de Gazebo (SDF) a partir de un mundo de mundo_core y del URDF, sin ROS.

Lo mismo que fisica_core hace para MuJoCo: los solidos de techo plano son
cajas y los inclinados, cajas giradas con la cara de arriba en su plano. El robot es la
caja del chasis y cuatro brazos. Cada oruga es una cadena de rodillos
(polea activa, polea de la punta y rodillos en los dos tramos) movidos por
JointController a la velocidad de banda: el TrackController de Gazebo no
mueve nada con el DART de Ubuntu (le faltan los parches de Gazebo; tampoco
el ejemplo tracked_vehicle_simple se mueve). Las ruedas de un lado
escuchan los mismos tres topicos: polea, punta y rodillos. Las juntas de
los brazos van por posicion con JointPositionController, rigidas como el
sinfin. La caja lleva la IMU y OdometryPublisher da la pose verdadera.
"""

from __future__ import annotations

import math
import os
from typing import List, Sequence, Tuple

MODELO = 'horu'
NOMBRES = ('fr', 'fl', 'rr', 'rl')
IZQUIERDA = (False, True, False, True)
LADOS = ('izq', 'der')
RUEDAS = ('polea', 'punta', 'rodillo')

TOPICO_ODOM = '/model/%s/odometry' % MODELO
TOPICO_IMU = '/imu'


def juntas() -> List[str]:
    return ['crawler_%s_joint' % n for n in NOMBRES]


def eslabones() -> List[str]:
    return ['crawler_%s_link' % n for n in NOMBRES]


def topico_rueda(lado: str, rueda: str) -> str:
    """Velocidad (rad/s) de las ruedas de un tipo en un lado."""
    return '/%s/banda_%s/%s' % (MODELO, lado, rueda)


def topico_brazo(i: int) -> str:
    # Sin el '/0/' de Gazebo: ROS no admite un tramo que empiece por cifra
    return '/model/%s/joint/%s/cmd_pos' % (MODELO, juntas()[i])


def puentes() -> List[str]:
    """Argumentos de ros_gz_bridge/parameter_bridge ('[' gz->ros, ']' ros->gz)."""
    out = [TOPICO_ODOM + '@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
           TOPICO_IMU + '@sensor_msgs/msg/Imu[ignition.msgs.IMU']
    for lado in LADOS:
        for rueda in RUEDAS:
            out.append(topico_rueda(lado, rueda) + '@std_msgs/msg/Float64]ignition.msgs.Double')
    for i in range(4):
        out.append(topico_brazo(i) + '@std_msgs/msg/Float64]ignition.msgs.Double')
    return out


def ruedas(s: float, r1: float, r2: float, largo: float,
           separacion: float = 0.05, radio: float = 0.03) -> List[Tuple[float, float, float, str]]:
    """(x, z, radio, tipo) de las ruedas de una oruga en el marco del brazo.

    s = +1 delantera (punta en +x), -1 trasera. La banda es tangente a las dos
    poleas; los rodillos la tocan desde dentro en los dos tramos, solapados
    para que un canto no se encaje entre dos."""
    out = [(0.0, 0.0, r1, 'polea'), (s * largo, 0.0, r2, 'punta')]
    rr = min(radio, 0.9 * r2)
    seno = (r1 - r2) / largo
    coseno = math.sqrt(1.0 - seno * seno)
    for nz in (-coseno, coseno):
        nx = s * seno
        t1 = (r1 * nx, r1 * nz)
        t2 = (s * largo + r2 * nx, r2 * nz)
        n = max(2, int(math.hypot(t2[0] - t1[0], t2[1] - t1[1]) // separacion))
        for k in range(1, n):
            f = k / n
            out.append((t1[0] + f * (t2[0] - t1[0]) - rr * nx,
                        t1[1] + f * (t2[1] - t1[1]) - rr * nz, rr, 'rodillo'))
    return out


def _n(v: Sequence[float]) -> str:
    return ' '.join('%.6g' % x for x in v)


def _inercia_caja(m: float, x: float, y: float, z: float) -> str:
    return ('<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            % (m, m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12))


def _rozamiento(mu: float) -> str:
    return ('<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface>'
            % (mu, mu))


def _rozamiento_rueda(mu_avance: float, mu_lateral: float) -> str:
    """Anisotropo: DART reparte el rozamiento en dos direcciones independientes
    (piramide), asi que con el mismo valor en las dos el robot no gira en
    parado: el agarre lateral pleno vence a la traccion. fdir1 es el eje del
    cilindro (z de su colision), el lateral."""
    return ('<surface><friction><ode><mu>%g</mu><mu2>%g</mu2><fdir1>0 0 1</fdir1>'
            '</ode></friction></surface>' % (mu_lateral, mu_avance))


def _material(r: float, g: float, b: float) -> str:
    return ('<material><ambient>%g %g %g 1</ambient><diffuse>%g %g %g 1</diffuse></material>'
            % (r, g, b, r, g, b))


def terreno_sdf(mundo, carpeta: str) -> str:
    """Modelo estatico con un eslabon por solido."""
    trozos = []
    for k, so in enumerate(getattr(mundo, 'solidos', ())):
        esquinas = so.esquinas()
        techos = [so.techo(x, y) for x, y in esquinas]
        if max(techos) < 0.002:
            continue
        color = so.color if so.color is not None else (0.62, 0.6, 0.55)
        if so.bx == 0.0 and so.by == 0.0:
            alto = so.a + 0.01
            geom = '<box><size>%s</size></box>' % _n((so.largo, so.ancho, alto))
            pose = _n((so.cx, so.cy, so.a - alto / 2, 0, 0, so.rumbo))
        else:
            # Inclinado: caja girada con la cara de arriba en el plano del
            # techo y hundida en el suelo. Con una malla, el ODE de DART se
            # cae al cargarla (OdeMesh::fillArrays).
            c, sn = math.cos(so.rumbo), math.sin(so.rumbo)
            gu = so.bx * c + so.by * sn            # pendiente a lo largo
            gv = -so.bx * sn + so.by * c           # y a lo ancho
            cab, bal = math.atan(gu), math.atan(gv)
            grosor = max(techos) + 0.3
            # Normal del techo y punto del techo en el centro
            nx, ny, nz = -so.bx, -so.by, 1.0
            norma = math.sqrt(nx * nx + ny * ny + nz * nz)
            zc = so.techo(so.cx, so.cy)
            geom = '<box><size>%s</size></box>' % _n(
                (so.largo / math.cos(cab), so.ancho / math.cos(bal), grosor))
            pose = _n((so.cx - grosor / 2 * nx / norma, so.cy - grosor / 2 * ny / norma,
                       zc - grosor / 2 * nz / norma, bal, -cab, so.rumbo))
        trozos.append(
            '<link name="s%d"><pose>%s</pose>'
            '<collision name="c"><geometry>%s</geometry>%s</collision>'
            '<visual name="v"><geometry>%s</geometry>%s</visual></link>'
            % (k, pose, geom, _rozamiento(1.0), geom, _material(*color)))
    if not trozos:
        return ''
    return '<model name="terreno"><static>true</static>%s</model>' % ''.join(trozos)


def robot_sdf(geo, x: float = 0.0, y: float = 0.0, rumbo: float = 0.0,
              z_suelo: float = 0.0, rozamiento: float = 0.9,
              rozamiento_lateral: float = 0.6,
              fuerza_banda: float = 350.0, par_brazo: float = 200.0) -> str:
    """El modelo del robot. fuerza_banda (N) es el tope de cada rueda: el del
    RMD en la polea, y lo mismo en cada rodillo."""
    o = geo.orugas
    largo_c, ancho_c, alto_c = geo.chasis
    w = geo.ancho_oruga
    partes = [
        '<link name="base_link">%s'
        '<collision name="caja"><geometry><box><size>%s</size></box></geometry>%s</collision>'
        '<visual name="caja"><geometry><box><size>%s</size></box></geometry>%s</visual>'
        '<sensor name="imu" type="imu"><pose>%s</pose><always_on>1</always_on>'
        '<update_rate>100</update_rate><topic>imu</topic></sensor></link>'
        % (_inercia_caja(geo.masa_chasis, largo_c, ancho_c, alto_c),
           _n(geo.chasis), _rozamiento(0.4), _n(geo.chasis), _material(0.8, 0.05, 0.05),
           _n((largo_c / 2 - 0.05, 0, alto_c / 2, 0, 0, 0)))]
    plugins = []
    cil = '<cylinder><radius>%g</radius><length>%g</length></cylinder>'
    for i, (px, py) in enumerate(o.pivotes):
        s = 1.0 if px > 0 else -1.0
        eslabon, junta = eslabones()[i], juntas()[i]
        lado = LADOS[0] if IZQUIERDA[i] else LADOS[1]
        lista = ruedas(s, o.radio_polea, o.radio_punta, o.largo)
        m_rueda = 0.4 / len(lista)
        # El brazo: chapa amarilla por fuera, sin choque (chocan las ruedas)
        partes.append(
            '<link name="%s"><pose relative_to="base_link">%s</pose>'
            '<inertial><pose>%s</pose><mass>%g</mass><inertia><ixx>0.02</ixx><iyy>0.05</iyy>'
            '<izz>0.05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            '<visual name="chapa"><pose>%s</pose><geometry><box><size>%s</size></box>'
            '</geometry>%s</visual></link>'
            '<joint name="%s" type="revolute"><parent>base_link</parent><child>%s</child>'
            '<axis><xyz>0 %g 0</xyz><limit><lower>-1.6</lower><upper>1.6</upper>'
            '<effort>%g</effort><velocity>1</velocity></limit>'
            '<dynamics><damping>2</damping></dynamics></axis></joint>'
            % (eslabon, _n((px, py, 0, 0, 0, 0)), _n((s * o.largo / 2, 0, 0, 0, 0, 0)),
               max(geo.masa_brazo - 0.4, 0.1),
               _n((s * o.largo / 2, (w / 2 + 0.004) * (1 if py > 0 else -1), 0, 0, 0, 0)),
               _n((o.largo, 0.006, 1.8 * o.radio_punta)), _material(1.0, 0.82, 0.0),
               junta, eslabon, -s, par_brazo))
        for j, (rx, rz, r, tipo) in enumerate(lista):
            nombre = '%s_r%d' % (eslabon, j)
            partes.append(
                '<link name="%s"><pose relative_to="%s">%s</pose>'
                '<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz>'
                '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
                '<collision name="c"><pose>0 0 0 %g 0 0</pose><geometry>%s</geometry>%s'
                '</collision>'
                '<visual name="v"><pose>0 0 0 %g 0 0</pose><geometry>%s</geometry>%s</visual>'
                '</link>'
                '<joint name="%s_j" type="revolute"><parent>%s</parent><child>%s</child>'
                '<axis><xyz>0 1 0</xyz><limit><effort>%g</effort></limit></axis></joint>'
                % (nombre, eslabon, _n((rx, 0, rz, 0, 0, 0)), m_rueda,
                   m_rueda * r * r / 2, m_rueda * r * r / 2, m_rueda * r * r / 2,
                   math.pi / 2, cil % (r, w), _rozamiento_rueda(rozamiento, rozamiento_lateral),
                   math.pi / 2, cil % (r, w), _material(0.07, 0.07, 0.07),
                   nombre, eslabon, nombre, fuerza_banda * r))
            plugins.append(
                '<plugin filename="ignition-gazebo-joint-controller-system" '
                'name="ignition::gazebo::systems::JointController">'
                '<joint_name>%s_j</joint_name><topic>%s</topic></plugin>'
                % (nombre, topico_rueda(lado, tipo)))
        # Por par con un PID duro: el sinfin no cede
        plugins.append(
            '<plugin filename="ignition-gazebo-joint-position-controller-system" '
            'name="ignition::gazebo::systems::JointPositionController">'
            '<joint_name>%s</joint_name><topic>%s</topic>'
            '<p_gain>2000</p_gain><i_gain>200</i_gain><d_gain>80</d_gain>'
            '<i_max>100</i_max><i_min>-100</i_min>'
            '<cmd_max>%g</cmd_max><cmd_min>%g</cmd_min></plugin>'
            % (junta, topico_brazo(i), par_brazo, -par_brazo))
    plugins.append(
        '<plugin filename="ignition-gazebo-joint-state-publisher-system" '
        'name="ignition::gazebo::systems::JointStatePublisher">%s</plugin>'
        % ''.join('<joint_name>%s</joint_name>' % j for j in juntas()))
    plugins.append(
        '<plugin filename="ignition-gazebo-odometry-publisher-system" '
        'name="ignition::gazebo::systems::OdometryPublisher">'
        '<odom_frame>odom</odom_frame><robot_base_frame>base_link</robot_base_frame>'
        '<dimensions>3</dimensions><odom_publish_frequency>50</odom_publish_frequency></plugin>')
    z = z_suelo + o.altura_reposo + 0.02
    return ('<model name="%s"><pose>%s</pose>%s%s</model>'
            % (MODELO, _n((x, y, z, 0, 0, rumbo)), ''.join(partes), ''.join(plugins)))


def mundo_sdf(mundo, geo, carpeta: str, x: float = 0.0, y: float = 0.0,
              rumbo: float = 0.0) -> str:
    """El SDF entero. Las mallas de las rampas se escriben en carpeta."""
    os.makedirs(carpeta, exist_ok=True)
    return ('<?xml version="1.0"?><sdf version="1.9"><world name="%s">'
            '<physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size>'
            '<real_time_factor>1</real_time_factor></physics>'
            '<plugin filename="ignition-gazebo-physics-system" '
            'name="ignition::gazebo::systems::Physics"/>'
            '<plugin filename="ignition-gazebo-user-commands-system" '
            'name="ignition::gazebo::systems::UserCommands"/>'
            '<plugin filename="ignition-gazebo-scene-broadcaster-system" '
            'name="ignition::gazebo::systems::SceneBroadcaster"/>'
            '<plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>'
            '<light type="directional" name="sol"><cast_shadows>true</cast_shadows>'
            '<pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>'
            '<direction>-0.4 0.2 -0.9</direction></light>'
            '<model name="suelo"><static>true</static><link name="l">'
            '<collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size>'
            '</plane></geometry>%s</collision>'
            '<visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size>'
            '</plane></geometry>%s</visual></link></model>%s%s</world></sdf>'
            % (MODELO, _rozamiento(1.0), _material(0.75, 0.75, 0.72),
               terreno_sdf(mundo, carpeta),
               robot_sdf(geo, x, y, rumbo, mundo.altura(x, y))))


def generar(urdf_xml: str, nombre_mundo: str, carpeta: str) -> str:
    """Escribe carpeta/mundo.sdf para ese mundo y ese URDF; devuelve la ruta."""
    from starcrawler_sim import contacto_core, mundo_core
    geo = contacto_core.geometria_robot_desde_urdf(urdf_xml)
    mundo = mundo_core.cargar_fichero(mundo_core.resolver_ruta(nombre_mundo))
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, 'mundo.sdf')
    with open(ruta, 'w', encoding='utf-8') as f:
        f.write(mundo_sdf(mundo, geo, carpeta))
    return ruta
