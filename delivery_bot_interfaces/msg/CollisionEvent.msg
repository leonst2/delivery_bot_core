# A collision reported by the simulation.
std_msgs/Header header
string robot_link    # URDF link that was hit
string other_object  # name of the other object
string other_tag     # tag of the other object
float32 impulse      # N*s
