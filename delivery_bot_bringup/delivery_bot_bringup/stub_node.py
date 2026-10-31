"""Placeholder for a stack node whose card hasn't landed yet: it only publishes heartbeats."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from delivery_bot_bringup.heartbeat import HeartbeatPublisher


def main():
    rclpy.init()
    node = Node('stub_node', automatically_declare_parameters_from_overrides=True)
    heartbeat = HeartbeatPublisher(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        del heartbeat
        node.destroy_node()
        rclpy.try_shutdown()
