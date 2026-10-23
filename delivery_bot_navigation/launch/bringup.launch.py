"""Bring up the shared navigation stack. sim:=true adds the simulation overlay (use_sim_time, remaps)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    nav_share = get_package_share_directory('delivery_bot_navigation')
    sim = LaunchConfiguration('sim').perform(context) == 'true'

    # The sim overlay is applied last so its values win. It may only hold simulation differences.
    params = [os.path.join(nav_share, 'config', 'nav2_params.yaml')]
    if sim:
        params.append(os.path.join(nav_share, 'config', 'sim.yaml'))

    description_launch = os.path.join(
        get_package_share_directory('delivery_bot'), 'launch', 'description.launch.py')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim_time': 'true' if sim else 'false'}.items(),
        ),
        Node(package='delivery_bot_grid_step', executable='grid_step_controller',
             name='grid_step_controller', output='screen', parameters=params),
        Node(package='delivery_bot_perception', executable='obstacle_tracker',
             name='obstacle_tracker', output='screen', parameters=params),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='true when running against the Unity simulation'),
        OpaqueFunction(function=_nodes),
    ])
