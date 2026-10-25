# Temporary copy of delivery_bot_interfaces/Heartbeat (delivery_bot_core#3). Delete this package when that lands.
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2

std_msgs/Header header
string node
uint8 status
string detail
