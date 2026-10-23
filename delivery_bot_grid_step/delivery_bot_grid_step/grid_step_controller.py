"""Grid-step controller (stub).

Target loop, implemented in leonst2/delivery_bot_sim#12:
HALT -> PERCEIVE -> UPDATE_MAP -> PLAN -> MOVE_ONE_STEP (step_size metres) -> HALT ...
For now the node only declares its parameters and reports IDLE on /grid_step/state (latched, published on change).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

STATES = ('IDLE', 'HALT', 'PERCEIVE', 'UPDATE_MAP', 'PLAN', 'MOVE_ONE_STEP', 'DONE', 'FAILED')


class GridStepController(Node):

    def __init__(self):
        super().__init__('grid_step_controller')
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_step_speed', 0.5)
        self.declare_parameter('settle_velocity', 0.02)
        self.declare_parameter('max_replan_failures', 3)

        # Latched, so late subscribers (e.g. the Unity UI) get the current state right away.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, '/grid_step/state', latched)
        self.state = None
        self.set_state('IDLE')
        self.get_logger().info(
            f"grid_step_controller stub up (step_size={self.get_parameter('step_size').value} m)")

    def set_state(self, state):
        if state not in STATES:
            raise ValueError(f'unknown state {state}')
        if state != self.state:
            self.state = state
            self._state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = GridStepController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
