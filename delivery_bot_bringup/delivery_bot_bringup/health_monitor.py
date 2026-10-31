"""Publishes the state of every stack node on /health (latched), based on /heartbeat."""

import time

from delivery_bot_interfaces.msg import Heartbeat
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from delivery_bot_bringup.health import HealthTracker, STALL_TIMEOUT
from delivery_bot_bringup.heartbeat import HEARTBEAT_TOPIC, HeartbeatPublisher

HEALTH_TOPIC = '/health'
HEALTH_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CHECK_PERIOD = 0.1  # s
REPUBLISH_PERIOD = 1.0  # s


class HealthMonitor(Node):

    def __init__(self):
        super().__init__('health_monitor')
        self.declare_parameter('expected_nodes', [''])
        self.declare_parameter('stall_timeout', STALL_TIMEOUT)
        expected = [name for name in self.get_parameter('expected_nodes').value if name]
        self._tracker = HealthTracker(expected, self.get_parameter('stall_timeout').value)
        self._last_snapshot = None
        self._last_publish = 0.0

        self._heartbeat = HeartbeatPublisher(self)
        self._publisher = self.create_publisher(DiagnosticArray, HEALTH_TOPIC, HEALTH_QOS)
        self.create_subscription(Heartbeat, HEARTBEAT_TOPIC, self._on_heartbeat, 50)
        # Steady clock, for the same reason as in HeartbeatPublisher.
        self._timer = self.create_timer(CHECK_PERIOD, self._check, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_heartbeat(self, message):
        self._tracker.update(message, time.monotonic())

    def _check(self):
        now = time.monotonic()
        snapshot = self._tracker.snapshot(now)
        if snapshot == self._last_snapshot and now - self._last_publish < REPUBLISH_PERIOD:
            return
        self._last_snapshot = snapshot
        self._last_publish = now

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [DiagnosticStatus(level=node.level, name=node.name, message=node.message)
                          for node in snapshot]
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
