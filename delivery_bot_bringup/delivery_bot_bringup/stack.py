"""The nodes of the stack, in start order.

Every entry is a stub until its card lands. The card then replaces package and executable in its row.
"""

from dataclasses import dataclass

STAGES = ('description', 'odometry', 'localization', 'perception', 'mapping', 'planning', 'executive')
OPTIONAL_STAGES = ('perception', 'mapping', 'planning', 'executive')
LOCALIZATION_MODES = ('mcl', 'ground_truth')


@dataclass(frozen=True)
class StackNode:
    name: str
    stage: str
    package: str = 'delivery_bot_bringup'
    executable: str = 'stub_node'
    respawn: bool = True  # restart after a crash
    localization_mode: str = ''  # only started in this localization mode


NODES = (
    StackNode('description_publisher', 'description'),  # core#5
    StackNode('odometry', 'odometry'),  # core#7
    StackNode('localization', 'localization', localization_mode='mcl'),  # core#8
    StackNode('ground_truth_localization', 'localization', localization_mode='ground_truth'),  # core#8
    StackNode('scan_filter', 'perception'),  # core#9
    StackNode('stereo', 'perception'),  # core#10
    StackNode('tracker', 'perception'),  # core#11
    StackNode('map_server', 'mapping'),  # core#12
    StackNode('occupancy', 'mapping'),  # core#13
    StackNode('costmap', 'mapping'),  # core#14
    StackNode('planner', 'planning'),  # core#17
    StackNode('step_controller', 'planning'),  # core#19
    StackNode('safety', 'planning'),  # core#20
    # Not restarted: a new executive doesn't know the interrupted mission, so a crash has to stay visible.
    StackNode('executive', 'executive', respawn=False),  # core#21
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
