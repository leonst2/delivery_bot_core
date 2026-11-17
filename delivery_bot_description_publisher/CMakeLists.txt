cmake_minimum_required(VERSION 3.8)
project(delivery_bot_description_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(delivery_bot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(description_publisher src/description_publisher_node.cpp)
target_compile_features(description_publisher PRIVATE cxx_std_17)
target_link_libraries(description_publisher rclcpp::rclcpp ${delivery_bot_interfaces_TARGETS})

install(TARGETS description_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
