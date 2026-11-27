cmake_minimum_required(VERSION 3.8)
project(delivery_bot_collision)

# Optimised by default: colcon passes no build type, and the pose checks have a throughput requirement (>= 1 M/s).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(delivery_bot_geometry REQUIRED)

# Plain C++ library without ROS dependencies, so the costmap, planner, controller and safety supervisor can use it
# in nodes and in tests alike.
add_library(${PROJECT_NAME} SHARED
  src/grid.cpp
  src/distance_field.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen delivery_bot_geometry::delivery_bot_geometry)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# No fused multiply-add contraction (GCC contracts by default on ARM64, not on x86_64): the same inputs must give
# the same answers on the robot, in the sim and in CI. Same as delivery_bot_geometry.
set(DELIVERY_BOT_COLLISION_FP_FLAGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(DELIVERY_BOT_COLLISION_FP_FLAGS -ffp-contract=off)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic ${DELIVERY_BOT_COLLISION_FP_FLAGS})
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 delivery_bot_geometry)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_grid.cpp
    test/test_distance_field.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_compile_options(test_${PROJECT_NAME} PRIVATE ${DELIVERY_BOT_COLLISION_FP_FLAGS})
endif()

ament_package()
