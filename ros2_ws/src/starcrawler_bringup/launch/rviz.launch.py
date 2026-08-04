"""
rviz.launch.py — visualizacion remota
=====================================
Para el portatil del operador: solo RViz y el robot_state_publisher, sin
tocar hardware. El PC de a bordo publica /joint_states por la red DDS.

    export ROS_DOMAIN_ID=<el mismo que el robot>
    ros2 launch starcrawler_bringup rviz.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descripcion = FindPackageShare('starcrawler_description')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [descripcion, 'urdf', 'starcrawler.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution(
                [descripcion, 'rviz', 'starcrawler.rviz'])],
        ),
    ])
