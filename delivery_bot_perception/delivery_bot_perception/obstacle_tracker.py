"""Dynamic obstacle tracker (stub).

Implemented in leonst2/delivery_bot_sim#13: cluster /scan, drop points explained by the static map, track
clusters across perception cycles and publish /tracked_obstacles (visualization_msgs/MarkerArray).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleTracker(Node):

    def __init__(self):
        super().__init__('obstacle_tracker')
        self._warned = False
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('obstacle_tracker stub up, listening on /scan')

    def _on_scan(self, scan):
        if not self._warned:
            self.get_logger().warn(f'Received /scan ({len(scan.ranges)} ranges); tracking not implemented yet')
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
