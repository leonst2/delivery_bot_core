"""robot_state_publisher for the delivery_bot URDF (publishes /tf_static and /robot_description)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('delivery_bot'), 'urdf', 'delivery_bot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (true in the Unity simulation)'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
