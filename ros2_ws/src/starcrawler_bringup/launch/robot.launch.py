"""
robot.launch.py — arranque completo de StarCrawler
==================================================
Levanta todo lo que corre en el PC de a bordo:

    robot_state_publisher  (URDF -> TF, dibuja el robot)
    starcrawler_bandas     (los tacos de las orugas giran con la banda)
    starcrawler_chasis     (el chasis apoyado en sus orugas)
    micro_ros_agent        (el ESP32 es un nodo ROS 2, como en Donatello)
    joy + starcrawler_teleop  (mando conectado al PC)

El estado del robot sale de uno solo de estos, por prioridad:
    sim:=true          starcrawler_sim, sin hardware
    simulate:=true     el driver serie con su ESP32 simulado
    micro_ros:=false   el driver serie con el ESP32 real (el camino anterior)
    (por defecto)      el agente de micro-ROS

Uso:
    ros2 launch starcrawler_bringup robot.launch.py port:=/dev/ttyUSB0
    ros2 launch starcrawler_bringup robot.launch.py sim:=true rviz:=true teleop:=false
    ros2 launch starcrawler_bringup robot.launch.py simulate:=true rviz:=true
    ros2 launch starcrawler_bringup robot.launch.py micro_ros:=false   # firmware serie
    ros2 launch starcrawler_bringup robot.launch.py sim:=true gui_mando:=true  # conducir desde la web
    ros2 launch starcrawler_bringup robot.launch.py sim:=true mundo:=escalon rviz:=true gui:=true
    ros2 launch starcrawler_bringup robot.launch.py port:=/dev/ttyUSB0 mundo:=rampa  # HW_SIMULADO
    ros2 launch starcrawler_bringup robot.launch.py sim:=true mundo:=escalera fisica:=true gui:=true  # MuJoCo
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PythonExpression,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def topes_del_mando():
    """Los topes del DS4 (ds4.yaml), para que la web use los mismos."""
    ruta = os.path.join(get_package_share_directory('starcrawler_teleop'),
                        'config', 'ds4.yaml')
    with open(ruta, encoding='utf-8') as f:
        p = yaml.safe_load(f)['starcrawler_teleop']['ros__parameters']
    return {k: float(p[k])
            for k in ('max_lineal', 'max_angular', 'factor_lento', 's_preset')}


def topes_de_velocidad(sim, vel_sim_dps):
    """max_lineal y max_angular del mando y de la web. Con sim:=true salen del
    tope del simulador (vel_sim_dps en la polea activa); sin el, los de
    ds4.yaml, que son los 40 dps del firmware."""
    topes = topes_del_mando()
    g = geometria_de_la_odometria()
    k_lineal = math.radians(1.0) * g['wheel_radius']           # m/s por dps
    k_angular = 2.0 * k_lineal / g['track_separation']         # rad/s por dps
    en_sim = ["'", sim, "'.lower() in ('true', '1')"]

    def tope(k, defecto):
        return ParameterValue(PythonExpression(
            ["(%r * float('" % k, vel_sim_dps, "')) if "] + en_sim
            + [' else %r' % defecto]), value_type=float)
    return {'max_lineal': tope(k_lineal, topes['max_lineal']),
            'max_angular': tope(k_angular, topes['max_angular'])}


def geometria_de_la_odometria():
    """Radio y separacion de odometry.yaml, para que el mundo integre igual."""
    ruta = os.path.join(get_package_share_directory('starcrawler_odometry'),
                        'config', 'odometry.yaml')
    with open(ruta, encoding='utf-8') as f:
        p = yaml.safe_load(f)['starcrawler_odometry']['ros__parameters']
    return {k: float(p[k]) for k in ('wheel_radius', 'track_separation')}


def generate_launch_description():
    descripcion = FindPackageShare('starcrawler_description')
    driver_share = FindPackageShare('starcrawler_driver')
    teleop_share = FindPackageShare('starcrawler_teleop')
    odometria_share = FindPackageShare('starcrawler_odometry')
    sim_share = FindPackageShare('starcrawler_sim')

    args = [
        DeclareLaunchArgument(
            'simulate', default_value='false',
            description='ESP32 simulado: permite probar sin hardware'),
        DeclareLaunchArgument(
            'port', default_value='/dev/starcrawler',
            description='Puerto serie del ESP32 (ver udev/99-starcrawler.rules)'),
        DeclareLaunchArgument(
            'micro_ros', default_value='true',
            description='El ESP32 es nodo ROS 2 nativo y se lanza el agente '
                        'de micro-ROS; false = driver serie (firmware '
                        'starcrawler_esp32_ros2)'),
        DeclareLaunchArgument(
            'micro_ros_baud', default_value='921600',
            description='Debe coincidir con SERIE_BAUDIOS del config.h de la '
                        'app de micro-ROS'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Robot simulado a nivel de topicos, sin hardware ni '
                        'protocolo serie. Sustituye al driver por completo'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Interfaz web en http://localhost:8000'),
        DeclareLaunchArgument(
            'gui_mando', default_value='false',
            description='Conducir desde la interfaz web (arranca la GUI). El '
                        'enlace con la clave sale en el log'),
        DeclareLaunchArgument(
            'gui_clave', default_value='',
            description='Clave del mando web; vacia = una al azar en cada '
                        'arranque'),
        DeclareLaunchArgument(
            'odom', default_value='true',
            description='Publicar odometria de orugas y la TF odom->base'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Arrancar el mando y la teleoperacion'),
        DeclareLaunchArgument(
            'joy_udp', default_value='false',
            description='El mando llega por UDP desde el puente de Windows '
                        '(tools/joy_bridge) en vez del nodo joy local'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Abrir RViz (solo si hay pantalla)'),
        DeclareLaunchArgument(
            'joy_device', default_value='0',
            description='Indice del mando para el nodo joy'),
        DeclareLaunchArgument(
            'mundo', default_value='',
            description='Mundo con obstaculos para el robot simulado: nombre '
                        'en starcrawler_sim/mundos o ruta a un .yaml. Vacio = '
                        'como hasta ahora'),
        DeclareLaunchArgument(
            'vel_sim_dps', default_value='80',
            description='Tope de las orugas con sim:=true, en dps de la polea '
                        'activa (80 = 0,107 m/s). El robot real va con el '
                        'del firmware (40)'),
        DeclareLaunchArgument(
            'fisica', default_value='false',
            description='Fisica de MuJoCo (fisica_node) en vez del contacto '
                        'cuasiestatico. Sin mundo:=, en llano'),
        DeclareLaunchArgument(
            'visor', default_value='false',
            description='Con fisica:=true, abrir tambien el visor de MuJoCo'),
    ]

    simulate = LaunchConfiguration('simulate')
    port = LaunchConfiguration('port')
    sim = LaunchConfiguration('sim')
    micro_ros = LaunchConfiguration('micro_ros')
    teleop = LaunchConfiguration('teleop')
    gui_mando = LaunchConfiguration('gui_mando')
    mundo = LaunchConfiguration('mundo')
    mux_yaml = PathJoinSubstitution([teleop_share, 'config', 'mux.yaml'])

    # Con mundo, mundo_node publica la pose verdadera (TF y juntas del chasis)
    fisica = PythonExpression([
        "'", LaunchConfiguration('fisica'), "'.lower() in ('true', '1')"])
    con_mundo = PythonExpression(["'", mundo, "' != '' or ", fisica])
    sin_mundo = PythonExpression(["not (", con_mundo, ")"])

    # Los muxes hacen falta con cualquier fuente de consignas. Como
    # IfCondition, 'true', 'True' y '1' valen lo mismo
    def si(valor):
        return ["'", valor, "'.lower() in ('true', '1')"]

    con_mux = PythonExpression(si(teleop) + [' or '] + si(gui_mando))
    con_gui = PythonExpression(si(LaunchConfiguration('gui')) + [' or ']
                               + si(gui_mando))

    # Solo una fuente de estado a la vez, por prioridad: sim, simulate,
    # driver serie y, si nada de eso, micro-ROS
    usar_driver = PythonExpression(
        ['not ('] + si(sim) + [') and (('] + si(simulate)
        + [') or not ('] + si(micro_ros) + ['))'])
    usar_agente = PythonExpression(
        ['not ('] + si(sim) + [') and not ('] + si(simulate)
        + [') and ('] + si(micro_ros) + [')'])

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [descripcion, 'urdf', 'starcrawler.urdf.xacro'])]),
        value_type=str)

    nodos = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        # Los tacos de las orugas se mueven con la velocidad de las bandas
        Node(
            package='starcrawler_odometry',
            executable='bandas_node',
            name='starcrawler_bandas',
            parameters=[{'wheel_radius': geometria_de_la_odometria()['wheel_radius']}],
            output='screen',
        ),
        # El driver habla con el ESP32 del firmware serie (o con su
        # simulador de puerto serie): el camino anterior a micro-ROS.
        Node(
            package='starcrawler_driver',
            executable='driver_node',
            name='starcrawler_driver',
            condition=IfCondition(usar_driver),
            parameters=[
                PathJoinSubstitution([driver_share, 'config', 'driver.yaml']),
                {'simulate': ParameterValue(simulate, value_type=bool),
                 'port': ParameterValue(port, value_type=str)},
            ],
            output='screen',
        ),
        # Agente de micro-ROS. Con el firmware nuevo el ESP32 publica y se
        # suscribe por si mismo, asi que el nodo driver sobra: el agente
        # solo hace de pasarela entre el puerto serie y el grafo.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            condition=IfCondition(usar_agente),
            arguments=['serial', '--dev', LaunchConfiguration('port'),
                       '-b', LaunchConfiguration('micro_ros_baud')],
            output='screen',
        ),
        # Robot simulado: habla los mismos topicos que hablara el ESP32
        # con micro-ROS, asi que el resto del grafo no lo distingue.
        Node(
            package='starcrawler_sim',
            executable='sim_node',
            name='starcrawler_sim',
            condition=IfCondition(sim),
            parameters=[PathJoinSubstitution(
                [sim_share, 'config', 'sim.yaml']),
                {'vel_max_dps': ParameterValue(
                    LaunchConfiguration('vel_sim_dps'), value_type=float)}],
            output='screen',
        ),
        # Odometria de orugas. Separada del driver: se prueba y se
        # sustituye sin tocarlo (ver starcrawler_odometry). Con mundo sigue
        # publicando /odom, pero la TF la da mundo_node.
        Node(
            package='starcrawler_odometry',
            executable='odometry_node',
            name='starcrawler_odometry',
            condition=IfCondition(LaunchConfiguration('odom')),
            parameters=[
                PathJoinSubstitution([odometria_share, 'config', 'odometry.yaml']),
                {'publish_tf': ParameterValue(sin_mundo, value_type=bool)},
            ],
            output='screen',
        ),
        # El chasis sobre sus orugas: juntas virtuales de altura, cabeceo
        # y balanceo a partir de las elevaciones. Sin el, base_link no
        # tiene TF: va siempre, salvo con mundo, que publica las mismas.
        Node(
            package='starcrawler_odometry',
            executable='chasis_node',
            name='starcrawler_chasis',
            condition=IfCondition(sin_mundo),
            output='screen',
        ),
        # El robot en un mundo con obstaculos: observa /starcrawler/state y
        # publica la pose verdadera sobre el terreno (ver starcrawler_sim)
        Node(
            package='starcrawler_sim',
            executable='mundo_node',
            name='starcrawler_mundo',
            condition=IfCondition(PythonExpression(
                ["'", mundo, "' != '' and not ", fisica])),
            parameters=[{'mundo': ParameterValue(mundo, value_type=str),
                         **geometria_de_la_odometria()}],
            output='screen',
        ),
        # Lo mismo con la fisica de MuJoCo
        Node(
            package='starcrawler_sim',
            executable='fisica_node',
            name='starcrawler_mundo',
            condition=IfCondition(fisica),
            parameters=[{'mundo': ParameterValue(PythonExpression(
                             ["'", mundo, "' or 'llano'"]), value_type=str),
                         'visor': ParameterValue(PythonExpression(
                             ["'", LaunchConfiguration('visor'),
                              "'.lower() in ('true', '1')"]), value_type=bool),
                         **geometria_de_la_odometria()}],
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('teleop'), "' == 'true' and '",
                LaunchConfiguration('joy_udp'), "' != 'true'"])),
            parameters=[{
                'device_id': ParameterValue(
                    LaunchConfiguration('joy_device'), value_type=int),
                'deadzone': 0.0,        # la zona muerta la aplica el teleop
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='starcrawler_teleop',
            executable='joy_udp_node',
            name='joy_udp_node',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('teleop'), "' == 'true' and '",
                LaunchConfiguration('joy_udp'), "' == 'true'"])),
            output='screen',
        ),
        Node(
            package='starcrawler_teleop',
            executable='teleop_node',
            name='starcrawler_teleop',
            condition=IfCondition(teleop),
            parameters=[
                PathJoinSubstitution([teleop_share, 'config', 'ds4.yaml']),
                topes_de_velocidad(sim, LaunchConfiguration('vel_sim_dps'))],
            # El mando entra a los muxes como dos fuentes: joy siempre (ceros
            # en reposo) y joy_activo solo mientras se toca (ver mux.yaml)
            remappings=[('cmd_vel', 'cmd_vel_joy'),
                        ('cmd_vel_activo', 'cmd_vel_joy_activo'),
                        ('crawler/command', 'crawler/command_joy'),
                        ('crawler/command_activo', 'crawler/command_joy_activo')],
            output='screen',
        ),
        # Multiplexores de velocidad y de orugas: eligen por prioridad y
        # descartan la fuente que se calla. Si caen, el ESP32 para solo.
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            condition=IfCondition(con_mux),
            parameters=[mux_yaml],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            respawn=True,
            respawn_delay=1.0,
            output='screen',
        ),
        Node(
            package='starcrawler_teleop',
            executable='crawler_mux',
            name='crawler_mux',
            condition=IfCondition(con_mux),
            parameters=[mux_yaml],
            respawn=True,
            respawn_delay=1.0,
            output='screen',
        ),
        # Interfaz web. Apagada por defecto: levanta un servidor HTTP y
        # durante el arranque automatico no siempre interesa.
        Node(
            package='starcrawler_gui',
            executable='gui_node',
            name='starcrawler_gui',
            condition=IfCondition(con_gui),
            parameters=[topes_del_mando(),
                        topes_de_velocidad(sim, LaunchConfiguration('vel_sim_dps')), {
                'mando': ParameterValue(gui_mando, value_type=bool),
                'mando_token': ParameterValue(
                    LaunchConfiguration('gui_clave'), value_type=str),
            }],
            output='screen',
        ),
        # Marco fijo odom: el robot se desplaza por la rejilla. El
        # starcrawler.rviz fija base_footprint y sirve para view_model.
        # Con mundo, mundo.rviz anade el terreno, el apoyo y la verdad.
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([
                descripcion, 'rviz', PythonExpression([
                    "'mundo.rviz' if '", mundo, "' != '' else 'plano.rviz'"])])],
        ),
    ]

    return LaunchDescription(args + nodos)
