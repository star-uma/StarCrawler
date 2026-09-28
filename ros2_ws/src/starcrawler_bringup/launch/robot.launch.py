"""
robot.launch.py — arranque completo de StarCrawler
==================================================
Levanta todo lo que corre en el PC de a bordo:

    robot_state_publisher  (URDF -> TF, dibuja el robot)
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
"""
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
    ]

    simulate = LaunchConfiguration('simulate')
    port = LaunchConfiguration('port')
    sim = LaunchConfiguration('sim')
    micro_ros = LaunchConfiguration('micro_ros')
    teleop = LaunchConfiguration('teleop')
    gui_mando = LaunchConfiguration('gui_mando')
    mux_yaml = PathJoinSubstitution([teleop_share, 'config', 'mux.yaml'])

    # Los muxes hacen falta con cualquier fuente de consignas
    con_mux = PythonExpression([
        "'", teleop, "' == 'true' or '", gui_mando, "' == 'true'"])
    con_gui = PythonExpression([
        "'", LaunchConfiguration('gui'), "' == 'true' or '", gui_mando,
        "' == 'true'"])

    # Solo una fuente de estado a la vez, por prioridad: sim, simulate,
    # driver serie y, si nada de eso, micro-ROS
    usar_driver = PythonExpression([
        "'", sim, "' != 'true' and ('", simulate, "' == 'true' or '",
        micro_ros, "' != 'true')"])
    usar_agente = PythonExpression([
        "'", sim, "' != 'true' and '", simulate, "' != 'true' and '",
        micro_ros, "' == 'true'"])

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
                [sim_share, 'config', 'sim.yaml'])],
            output='screen',
        ),
        # Odometria de orugas. Separada del driver: se prueba y se
        # sustituye sin tocarlo (ver starcrawler_odometry).
        Node(
            package='starcrawler_odometry',
            executable='odometry_node',
            name='starcrawler_odometry',
            condition=IfCondition(LaunchConfiguration('odom')),
            parameters=[PathJoinSubstitution(
                [odometria_share, 'config', 'odometry.yaml'])],
            output='screen',
        ),
        # El chasis sobre sus orugas: juntas virtuales de altura, cabeceo
        # y balanceo a partir de las elevaciones. Sin el, base_link no
        # tiene TF: va siempre.
        Node(
            package='starcrawler_odometry',
            executable='chasis_node',
            name='starcrawler_chasis',
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
                PathJoinSubstitution([teleop_share, 'config', 'ds4.yaml'])],
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
            parameters=[topes_del_mando(), {
                'mando': ParameterValue(gui_mando, value_type=bool),
                'mando_token': ParameterValue(
                    LaunchConfiguration('gui_clave'), value_type=str),
            }],
            output='screen',
        ),
        # Marco fijo odom: el robot se desplaza por la rejilla. El
        # starcrawler.rviz fija base_footprint y sirve para view_model.
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution(
                [descripcion, 'rviz', 'plano.rviz'])],
        ),
    ]

    return LaunchDescription(args + nodos)
