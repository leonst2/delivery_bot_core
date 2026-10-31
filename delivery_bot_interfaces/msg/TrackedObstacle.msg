# One tracked dynamic obstacle. The frame comes from the TrackedObstacleArray header (map).
uint32 id
geometry_msgs/Point position       # m
geometry_msgs/Vector3 velocity     # m/s
float32 radius                     # m
float32[4] position_covariance     # m^2, row-major 2x2 covariance of x and y
geometry_msgs/Point predicted_position  # m, at prediction_horizon
float32 prediction_horizon         # s
float32 age                        # s since the track started
