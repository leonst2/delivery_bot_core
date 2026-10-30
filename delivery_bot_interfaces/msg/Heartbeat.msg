# Published by every stack node on /heartbeat at >= 2 Hz.
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2

std_msgs/Header header
string node    # node name
uint8 status   # one of the constants above
string detail
