"""
view_model.launch.py
====================
Comprueba el URDF sin robot: abre RViz con sliders para mover las cuatro
orugas a mano. Es la forma rapida de validar la cinematica y los signos.

    ros2 launch starcrawler_description view_model.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('starcrawler_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'starcrawler.urdf.xacro'])
    rviz_file = PathJoinSubstitution([pkg, 'rviz', 'starcrawler.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
        ),
    ])
