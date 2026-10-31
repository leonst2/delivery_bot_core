"""Exits 0 once every given node has sent a heartbeat, 1 after the timeout. Orders the stages in stack.launch.py."""

import argparse
import sys
import time

from delivery_bot_interfaces.msg import Heartbeat
import rclpy
from rclpy.node import Node

from delivery_bot_bringup.heartbeat import HEARTBEAT_TOPIC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('nodes', nargs='+')
    parser.add_argument('--timeout', type=float, default=30.0, help='seconds to wait')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('wait_for_heartbeats')
    missing = set(args.nodes)
    node.create_subscription(Heartbeat, HEARTBEAT_TOPIC, lambda message: missing.discard(message.node), 50)
    deadline = time.monotonic() + args.timeout
    try:
        while missing and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if missing:
        print(f'no heartbeat from {sorted(missing)} after {args.timeout} s', file=sys.stderr)
    return 1 if missing else 0
