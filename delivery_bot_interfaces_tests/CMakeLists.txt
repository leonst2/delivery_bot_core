cmake_minimum_required(VERSION 3.8)
project(delivery_bot_interfaces_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(delivery_bot_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest(test_serialization_cpp test/test_serialization.cpp)
  target_link_libraries(test_serialization_cpp
    rclcpp::rclcpp
    ${delivery_bot_interfaces_TARGETS}
  )

  ament_add_pytest_test(test_serialization_py test/test_serialization.py)
endif()

ament_package()
