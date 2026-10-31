from delivery_bot_bringup.health import HealthTracker
from delivery_bot_interfaces.msg import Heartbeat
from diagnostic_msgs.msg import DiagnosticStatus


def heartbeat(node, status=Heartbeat.OK, detail=''):
    return Heartbeat(node=node, status=status, detail=detail)


def test_expected_node_without_heartbeat_is_stale():
    tracker = HealthTracker(['planner'])
    assert tracker.snapshot(0.0)[0].level == DiagnosticStatus.STALE


def test_recent_heartbeat_is_ok():
    tracker = HealthTracker(['planner'])
    tracker.update(heartbeat('planner'), 10.0)
    assert tracker.snapshot(10.9)[0].level == DiagnosticStatus.OK


def test_node_is_stalled_after_the_timeout():
    tracker = HealthTracker(['planner'], stall_timeout=1.0)
    tracker.update(heartbeat('planner'), 10.0)
    node = tracker.snapshot(11.1)[0]
    assert node.level == DiagnosticStatus.STALE
    assert node.message == 'stalled'


def test_stalled_node_recovers():
    tracker = HealthTracker(['planner'], stall_timeout=1.0)
    tracker.update(heartbeat('planner'), 10.0)
    tracker.update(heartbeat('planner'), 20.0)
    assert tracker.snapshot(20.1)[0].level == DiagnosticStatus.OK


def test_status_and_detail_are_reported():
    tracker = HealthTracker(['planner'])
    tracker.update(heartbeat('planner', Heartbeat.WARN, 'slow'), 10.0)
    node = tracker.snapshot(10.0)[0]
    assert node.level == DiagnosticStatus.WARN
    assert node.message == 'slow'


def test_unexpected_nodes_are_ignored_and_order_is_fixed():
    tracker = HealthTracker(['b', 'a'])
    tracker.update(heartbeat('other'), 10.0)
    assert [node.name for node in tracker.snapshot(10.0)] == ['a', 'b']
