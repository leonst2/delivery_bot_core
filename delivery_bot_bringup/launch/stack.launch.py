"""Starts the whole stack, stage by stage. sim:=true adds the simulation overlay (params/sim.yaml).

Each stage starts when every node of the previous stage has sent a heartbeat, not after a fixed delay.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from delivery_bot_bringup.stack import LOCALIZATION_MODES, OPTIONAL_STAGES, select_stages

STAGE_TIMEOUT = 30.0  # s


def _node_action(node, params):
    return Node(package=node.package, executable=node.executable, name=node.name, output='screen',
                parameters=params, respawn=node.respawn, respawn_delay=1.0)


def _start_stage(stages, index, params):
    """Actions for stage `index`: its nodes, plus a gate that starts the next stage once they're alive."""
    stage, nodes = stages[index]
    actions = [LogInfo(msg=f'Starting stage {stage}')]
    actions += [_node_action(node, params) for node in nodes]
    if index + 1 == len(stages):
        return actions

    gate = Node(package='delivery_bot_bringup', executable='wait_for_heartbeats', name=f'wait_for_{stage}',
                output='screen', arguments=[*(node.name for node in nodes), '--timeout', str(STAGE_TIMEOUT)])

    def on_gate_exit(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f'stage {stage} did not come up'))]
        return _start_stage(stages, index + 1, params)

    actions += [gate, RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=on_gate_exit))]
    return actions


def _stack(context):
    share = get_package_share_directory('delivery_bot_bringup')
    sim = LaunchConfiguration('sim').perform(context) == 'true'

    # The sim overlay is applied last so its values win. It may only hold simulation differences.
    params = [os.path.join(share, 'params', 'common.yaml')]
    if sim:
        params.append(os.path.join(share, 'params', 'sim.yaml'))
    params.append({'map_path': LaunchConfiguration('map').perform(context)})

    disabled = [stage for stage in OPTIONAL_STAGES if LaunchConfiguration(stage).perform(context) != 'true']
    stages = select_stages(LaunchConfiguration('localization').perform(context), disabled)

    expected = [node.name for _, nodes in stages for node in nodes]
    monitor = Node(package='delivery_bot_bringup', executable='health_monitor', name='health_monitor',
                   output='screen', parameters=[*params, {'expected_nodes': expected}], respawn=True,
                   respawn_delay=1.0)
    return [monitor, *_start_stage(stages, 0, params)]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='true when running against the Unity simulation'),
        DeclareLaunchArgument('map', default_value='', description='path of the static map'),
        DeclareLaunchArgument('localization', default_value='mcl', choices=list(LOCALIZATION_MODES),
                              description='ground_truth is for simulation runs only'),
    ]
    arguments += [DeclareLaunchArgument(stage, default_value='true', choices=['true', 'false'],
                                        description=f'start the {stage} stage') for stage in OPTIONAL_STAGES]
    return LaunchDescription([*arguments, OpaqueFunction(function=_stack)])
