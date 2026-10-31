"""Tracks which nodes are alive. Takes plain timestamps, so it can be tested without a running node."""

from dataclasses import dataclass

from delivery_bot_interfaces.msg import Heartbeat
from diagnostic_msgs.msg import DiagnosticStatus

STALL_TIMEOUT = 1.0  # s without a heartbeat before a node counts as stalled

STATUS_TO_LEVEL = {
    Heartbeat.OK: DiagnosticStatus.OK,
    Heartbeat.WARN: DiagnosticStatus.WARN,
    Heartbeat.ERROR: DiagnosticStatus.ERROR,
}


@dataclass(frozen=True)
class NodeHealth:
    name: str
    level: bytes
    message: str


class HealthTracker:

    def __init__(self, expected_nodes, stall_timeout=STALL_TIMEOUT):
        self._expected = sorted(expected_nodes)
        self._stall_timeout = stall_timeout
        self._last = {}

    def update(self, heartbeat, now):
        if heartbeat.node in self._expected:
            self._last[heartbeat.node] = (now, heartbeat.status, heartbeat.detail)

    def snapshot(self, now):
        """Return the state of every expected node, sorted by name."""
        return [self._health(name, now) for name in self._expected]

    def _health(self, name, now):
        if name not in self._last:
            return NodeHealth(name, DiagnosticStatus.STALE, 'no heartbeat received yet')
        last_time, status, detail = self._last[name]
        if now - last_time > self._stall_timeout:
            return NodeHealth(name, DiagnosticStatus.STALE, 'stalled')
        return NodeHealth(name, STATUS_TO_LEVEL.get(status, DiagnosticStatus.ERROR), detail)
