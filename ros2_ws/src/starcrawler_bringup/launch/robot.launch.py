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
    ros2 launch starcrawler_bringup robot.launch.py port:=/dev/ttyUSB0
    ros2 launch starcrawler_bringup robot.launch.py teleop:=false   # solo driver
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descripcion = FindPackageShare('starcrawler_description')
    driver_share = FindPackageShare('starcrawler_driver')
    teleop_share = FindPackageShare('starcrawler_teleop')

    args = [
        DeclareLaunchArgument(
            'simulate', default_value='false',
            description='ESP32 simulado: permite probar sin hardware'),
        DeclareLaunchArgument(
            'port', default_value='/dev/starcrawler',
            description='Puerto serie del ESP32 (ver udev/99-starcrawler.rules)'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Arrancar el mando y la teleoperacion'),
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
        Node(
            package='starcrawler_driver',
            executable='driver_node',
            name='starcrawler_driver',
            parameters=[
                PathJoinSubstitution([driver_share, 'config', 'driver.yaml']),
                {'simulate': ParameterValue(simulate, value_type=bool),
                 'port': ParameterValue(port, value_type=str)},
            ],
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[{
                'device_id': ParameterValue(
                    LaunchConfiguration('joy_device'), value_type=int),
                'deadzone': 0.0,        # la zona muerta la aplica el teleop
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='starcrawler_teleop',
            executable='teleop_node',
            name='starcrawler_teleop',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[
                PathJoinSubstitution([teleop_share, 'config', 'ds4.yaml'])],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution(
                [descripcion, 'rviz', 'starcrawler.rviz'])],
        ),
    ]

    return LaunchDescription(args + nodos)
