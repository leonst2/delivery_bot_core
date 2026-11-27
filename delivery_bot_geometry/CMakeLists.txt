cmake_minimum_required(VERSION 3.8)
project(delivery_bot_geometry)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Plain C++ library without ROS dependencies (Eigen only), so planner, controller and collision checker can use it
# in nodes and in tests alike.
add_library(${PROJECT_NAME} SHARED
  src/angle.cpp
  src/se2.cpp
  src/path.cpp
  src/curve_util.cpp
  src/dubins.cpp
  src/reeds_shepp.cpp
  src/primitives.cpp
  src/arc.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# No fused multiply-add contraction: GCC contracts a * b + c by default on ARM64 but not on x86_64, which changes the
# last bits of every result. Planner paths must be identical on the robot, in the sim and in CI.
set(DELIVERY_BOT_GEOMETRY_FP_FLAGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(DELIVERY_BOT_GEOMETRY_FP_FLAGS -ffp-contract=off)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic ${DELIVERY_BOT_GEOMETRY_FP_FLAGS})
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_angle.cpp
    test/test_se2.cpp
    test/test_path.cpp
    test/test_dubins.cpp
    test/test_reeds_shepp.cpp
    test/test_primitives.cpp
    test/test_arc.cpp
    test/test_vectors.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_compile_options(test_${PROJECT_NAME} PRIVATE ${DELIVERY_BOT_GEOMETRY_FP_FLAGS})
  # Shared with the Rust port (rust/tests/vectors.rs).
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    ARC_VECTORS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/vectors/arc_vectors.txt")

  # Writes test/vectors/arc_vectors.txt (see the file header). Built with the tests, not installed.
  add_executable(generate_arc_vectors test/generate_arc_vectors.cpp)
  target_link_libraries(generate_arc_vectors ${PROJECT_NAME})
  target_compile_options(generate_arc_vectors PRIVATE ${DELIVERY_BOT_GEOMETRY_FP_FLAGS})
endif()

ament_package()
