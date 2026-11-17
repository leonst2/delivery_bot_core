cmake_minimum_required(VERSION 3.8)
project(delivery_bot_scan_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(delivery_bot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(scan_filter src/scan_filter_node.cpp)
target_compile_features(scan_filter PRIVATE cxx_std_17)
target_link_libraries(scan_filter rclcpp::rclcpp ${delivery_bot_interfaces_TARGETS})

install(TARGETS scan_filter DESTINATION lib/${PROJECT_NAME})

ament_package()
