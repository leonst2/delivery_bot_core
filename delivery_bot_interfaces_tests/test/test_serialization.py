from delivery_bot_interfaces.msg import AckermannCommand, Heartbeat
from delivery_bot_interfaces.srv import ResetSim
from rclpy.serialization import deserialize_message, serialize_message


def round_trip(message):
    return deserialize_message(serialize_message(message), type(message))


def test_ackermann_command_round_trip():
    message = AckermannCommand(speed=1.5, steering_angle=-0.25)
    assert round_trip(message) == message


def test_heartbeat_round_trip():
    message = Heartbeat(node='planner', status=Heartbeat.WARN, detail='slow')
    assert round_trip(message) == message


def test_service_request_round_trip():
    message = ResetSim.Request(seed=42)
    message.robot_pose.position.x = 2.0
    assert round_trip(message) == message
