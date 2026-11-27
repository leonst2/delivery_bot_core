cmake_minimum_required(VERSION 3.8)
project(delivery_bot_planner)

# Optimised by default: colcon passes no build type, and planning has time limits (< 1 s for 95 % of queries).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(delivery_bot_geometry REQUIRED)
find_package(delivery_bot_collision REQUIRED)
find_package(delivery_bot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# No fused multiply-add contraction: byte-identical paths on ARM64 and x86_64 (same as the libraries).
set(DELIVERY_BOT_PLANNER_FLAGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(DELIVERY_BOT_PLANNER_FLAGS -Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

# The search itself, without ROS: tested on its own.
add_library(planner_core SHARED
  src/heuristic.cpp
  src/hybrid_astar.cpp
)
target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(planner_core PUBLIC
  Eigen3::Eigen
  delivery_bot_geometry::delivery_bot_geometry
  delivery_bot_collision::delivery_bot_collision
)
target_compile_features(planner_core PUBLIC cxx_std_17)
target_compile_options(planner_core PRIVATE ${DELIVERY_BOT_PLANNER_FLAGS})

add_executable(planner src/planner_node.cpp)
target_compile_features(planner PRIVATE cxx_std_17)
target_compile_options(planner PRIVATE ${DELIVERY_BOT_PLANNER_FLAGS})
target_link_libraries(planner rclcpp::rclcpp ${delivery_bot_interfaces_TARGETS})

install(TARGETS planner_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS planner DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planner_core
    test/test_heuristic.cpp
    test/test_hybrid_astar.cpp
    TIMEOUT 300
  )
  target_link_libraries(test_planner_core planner_core)
  target_compile_options(test_planner_core PRIVATE ${DELIVERY_BOT_PLANNER_FLAGS})
endif()

ament_package()
