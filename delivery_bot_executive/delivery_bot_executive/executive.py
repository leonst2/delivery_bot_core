"""Grid-step executive (delivery_bot_core#21, #22).

Stub: only publishes heartbeats, so the stack and its health monitor run. The card replaces it.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from delivery_bot_bringup.heartbeat import HeartbeatPublisher


def main():
    rclpy.init()
    node = Node('executive')
    heartbeat = HeartbeatPublisher(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        del heartbeat
        node.destroy_node()
        rclpy.try_shutdown()
