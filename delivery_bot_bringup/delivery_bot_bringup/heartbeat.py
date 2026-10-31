"""Heartbeat helper for Python nodes. C++ and Rust nodes publish the same message with about 10 lines of code."""

from delivery_bot_interfaces.msg import Heartbeat
from rclpy.clock import Clock, ClockType

HEARTBEAT_TOPIC = '/heartbeat'
DEFAULT_PERIOD = 0.25  # s


class HeartbeatPublisher:
    """Publishes the node's status on /heartbeat. Call set_status() when it changes.

    The timer uses the steady clock on purpose. With use_sim_time, ROS timers stop without /clock, so a healthy
    node would look stalled until Unity enters Play mode, and again whenever it pauses.
    """

    def __init__(self, node, period=None):
        if period is None:
            if not node.has_parameter('heartbeat_period'):
                node.declare_parameter('heartbeat_period', DEFAULT_PERIOD)
            period = node.get_parameter('heartbeat_period').value
        self._node = node
        self._status = Heartbeat.OK
        self._detail = ''
        self._publisher = node.create_publisher(Heartbeat, HEARTBEAT_TOPIC, 10)
        self._timer = node.create_timer(period, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_status(self, status, detail=''):
        self._status = status
        self._detail = detail

    def _publish(self):
        message = Heartbeat(node=self._node.get_name(), status=self._status, detail=self._detail)
        message.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(message)
