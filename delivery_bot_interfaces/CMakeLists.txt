cmake_minimum_required(VERSION 3.8)
project(delivery_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# In the dev image, rosidl_generator_rs (ros2_rust overlay) adds Rust bindings next to C/C++/Python.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AckermannCommand.msg"
  "msg/CollisionEvent.msg"
  "msg/GridStepState.msg"
  "msg/Heartbeat.msg"
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "srv/LoadMap.srv"
  "srv/LoadScenario.srv"
  "srv/ResetSim.srv"
  "action/ExecuteStep.action"
  "action/NavigateToGoal.action"
  "action/PlanPath.action"
  DEPENDENCIES action_msgs geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
