cmake_minimum_required(VERSION 3.8)
project(delivery_bot_spike_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# In the dev image, rosidl_generator_rs (ros2_rust overlay) adds Rust bindings next to C/C++/Python.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
