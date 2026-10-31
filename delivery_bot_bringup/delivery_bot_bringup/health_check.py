"""Exits 0 once /health reports every node as OK. For smoke tests: ros2 run delivery_bot_bringup health_check."""

import argparse
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node

from delivery_bot_bringup.health_monitor import HEALTH_QOS, HEALTH_TOPIC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0, help='seconds to wait')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('health_check')
    latest = DiagnosticArray()

    def on_health(message):
        nonlocal latest
        latest = message

    node.create_subscription(DiagnosticArray, HEALTH_TOPIC, on_health, HEALTH_QOS)
    deadline = time.monotonic() + args.timeout
    healthy = False
    while time.monotonic() < deadline and not healthy:
        rclpy.spin_once(node, timeout_sec=0.1)
        healthy = bool(latest.status) and all(status.level == DiagnosticStatus.OK for status in latest.status)

    for status in latest.status:
        print(f'{status.name}: level {int.from_bytes(status.level, "big")} {status.message}')
    print('all nodes OK' if healthy else f'not healthy after {args.timeout} s')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if healthy else 1
