# State of the grid-step loop.
uint8 IDLE=0
uint8 HALT=1
uint8 PERCEIVE=2
uint8 UPDATE_MAP=3
uint8 PLAN=4
uint8 YIELD=5
uint8 MOVE_ONE_STEP=6
uint8 DONE=7
uint8 FAILED=8

std_msgs/Header header
uint8 state        # one of the constants above
string state_name  # the constant's name, e.g. "PLAN"
uint32 step_index
uint32 replans
uint32 yields
geometry_msgs/PoseStamped goal
string detail
