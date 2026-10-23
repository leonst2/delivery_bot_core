"""Grid-step controller (stub).

Target loop, implemented in leonst2/delivery_bot_sim#12:
HALT -> PERCEIVE -> UPDATE_MAP -> PLAN -> MOVE_ONE_STEP (step_size metres) -> HALT ...
For now the node only declares its parameters and reports IDLE on /grid_step/state.
"""

import rclpy
from rclpy.node import Node
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

        self.state = 'IDLE'
        self._state_pub = self.create_publisher(String, '/grid_step/state', 10)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info(
            f"grid_step_controller stub up (step_size={self.get_parameter('step_size').value} m)")

    def _publish_state(self):
        self._state_pub.publish(String(data=self.state))


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
