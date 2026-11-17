cmake_minimum_required(VERSION 3.8)
project(delivery_bot_occupancy)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(delivery_bot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(occupancy src/occupancy_node.cpp)
target_compile_features(occupancy PRIVATE cxx_std_17)
target_link_libraries(occupancy rclcpp::rclcpp ${delivery_bot_interfaces_TARGETS})

install(TARGETS occupancy DESTINATION lib/${PROJECT_NAME})

ament_package()
