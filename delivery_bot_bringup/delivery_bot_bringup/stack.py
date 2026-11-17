"""The nodes of the stack, in start order.

Every node lives in its own package. Until its card lands, the package's executable is a stub that only publishes
heartbeats.
"""

from dataclasses import dataclass

STAGES = ('description', 'odometry', 'localization', 'perception', 'mapping', 'planning', 'executive')
OPTIONAL_STAGES = ('perception', 'mapping', 'planning', 'executive')
LOCALIZATION_MODES = ('mcl', 'ground_truth')


@dataclass(frozen=True)
class StackNode:
    name: str
    stage: str
    package: str
    executable: str
    respawn: bool = True  # restart after a crash
    localization_mode: str = ''  # only started in this localization mode


NODES = (
    StackNode('description_publisher', 'description', 'delivery_bot_description_publisher',
              'description_publisher'),  # core#5
    StackNode('odometry', 'odometry', 'delivery_bot_odometry', 'odometry'),  # core#7
    StackNode('localization', 'localization', 'delivery_bot_localization', 'localization',
              localization_mode='mcl'),  # core#8
    StackNode('ground_truth_localization', 'localization', 'delivery_bot_localization', 'localization',
              localization_mode='ground_truth'),  # core#8
    StackNode('scan_filter', 'perception', 'delivery_bot_scan_filter', 'scan_filter'),  # core#9
    StackNode('stereo', 'perception', 'delivery_bot_stereo', 'stereo'),  # core#10
    StackNode('tracker', 'perception', 'delivery_bot_tracker', 'tracker'),  # core#11
    StackNode('map_server', 'mapping', 'delivery_bot_map_server', 'map_server'),  # core#12
    StackNode('occupancy', 'mapping', 'delivery_bot_occupancy', 'occupancy'),  # core#13
    StackNode('costmap', 'mapping', 'delivery_bot_costmap', 'costmap'),  # core#14
    StackNode('planner', 'planning', 'delivery_bot_planner', 'planner'),  # core#17
    StackNode('step_controller', 'planning', 'delivery_bot_step_controller', 'step_controller'),  # core#19
    StackNode('safety', 'planning', 'delivery_bot_safety', 'safety'),  # core#20
    # Not restarted: a new executive doesn't know the interrupted mission, so a crash has to stay visible.
    StackNode('executive', 'executive', 'delivery_bot_executive', 'executive', respawn=False),  # core#21
)


def select_stages(localization='mcl', disabled_stages=()):
    """Return [(stage, nodes)] in start order, without disabled stages and stages that have no node."""
    if localization not in LOCALIZATION_MODES:
        raise ValueError(f'unknown localization mode {localization}')
    unknown = set(disabled_stages) - set(OPTIONAL_STAGES)
    if unknown:
        raise ValueError(f'these stages can not be disabled: {sorted(unknown)}')

    stages = []
    for stage in STAGES:
        nodes = [node for node in NODES
                 if node.stage == stage and node.localization_mode in ('', localization)]
        if stage not in disabled_stages and nodes:
            stages.append((stage, nodes))
    return stages
