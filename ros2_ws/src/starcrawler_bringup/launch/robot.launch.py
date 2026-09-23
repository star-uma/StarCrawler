"""
robot.launch.py — arranque completo de StarCrawler
==================================================
Levanta todo lo que corre en el PC de a bordo:

    robot_state_publisher  (URDF -> TF, dibuja el robot)
    starcrawler_driver     (puente serie con el ESP32)
    joy + starcrawler_teleop  (mando conectado al PC)

Uso:
    ros2 launch starcrawler_bringup robot.launch.py
    ros2 launch starcrawler_bringup robot.launch.py simulate:=true rviz:=true
    ros2 launch starcrawler_bringup robot.launch.py sim:=true rviz:=true teleop:=false
    ros2 launch starcrawler_bringup robot.launch.py port:=/dev/ttyUSB0
    ros2 launch starcrawler_bringup robot.launch.py teleop:=false   # solo driver
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration, PythonExpression,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
            'micro_ros', default_value='false',
            description='El ESP32 es nodo ROS 2 nativo: se lanza el agente '
                        'de micro-ROS en vez del nodo driver'),
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
        # El driver habla con el ESP32 real (o con su simulador de puerto
        # serie). Con sim:=true no pinta nada: el nodo de abajo publica el
        # estado directamente.
        Node(
            package='starcrawler_driver',
            executable='driver_node',
            name='starcrawler_driver',
            condition=UnlessCondition(PythonExpression([
                "'", LaunchConfiguration('sim'), "' == 'true' or '",
                LaunchConfiguration('micro_ros'), "' == 'true'"])),
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
            condition=IfCondition(LaunchConfiguration('micro_ros')),
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
            condition=IfCondition(LaunchConfiguration('sim')),
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
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[
                PathJoinSubstitution([teleop_share, 'config', 'ds4.yaml'])],
            # El mando ya no manda directo: entra al mux como una fuente mas
            remappings=[('cmd_vel', 'cmd_vel_joy')],
            output='screen',
        ),
        # Multiplexor de velocidad. Elige entre las fuentes por prioridad y
        # descarta la que se quede sin publicar (ver twist_mux.yaml).
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[
                PathJoinSubstitution([teleop_share, 'config', 'twist_mux.yaml'])],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            output='screen',
        ),
        # Interfaz web. Apagada por defecto: levanta un servidor HTTP y
        # durante el arranque automatico no siempre interesa.
        Node(
            package='starcrawler_gui',
            executable='gui_node',
            name='starcrawler_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
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
