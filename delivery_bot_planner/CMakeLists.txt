cmake_minimum_required(VERSION 3.8)
project(delivery_bot_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(delivery_bot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(planner src/planner_node.cpp)
target_compile_features(planner PRIVATE cxx_std_17)
target_link_libraries(planner rclcpp::rclcpp ${delivery_bot_interfaces_TARGETS})

install(TARGETS planner DESTINATION lib/${PROJECT_NAME})

ament_package()
