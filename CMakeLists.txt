cmake_minimum_required(VERSION 3.8)
project(ros2_uav_parameters)

set(CMAKE_CXX_STANDARD 23)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_uav_interfaces REQUIRED)

# Auto ROS Parameters Library
add_library(auto_ros_parameters SHARED
  src/auto_ros_parameters/parameters.cpp
  src/auto_ros_parameters/yaml_parameter_parser.cpp
  src/auto_ros_parameters/parameter_client.cpp)
target_include_directories(auto_ros_parameters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_uav_parameters>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(auto_ros_parameters yaml-cpp)
ament_target_dependencies(auto_ros_parameters rclcpp ros2_uav_interfaces)

# Parameter Server Node
add_executable(parameter_server src/parameter_server.cpp)
target_link_libraries(parameter_server auto_ros_parameters)

# Examples
add_executable(parameters_usage examples/parameters_usage.cpp)
target_link_libraries(parameters_usage auto_ros_parameters)
add_executable(yaml_parser_usage examples/yaml_parser_usage.cpp)
target_link_libraries(yaml_parser_usage auto_ros_parameters)
add_executable(parameter_client examples/parameter_client.cpp)
target_link_libraries(parameter_client auto_ros_parameters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_yaml_parser test/test_yaml_parameter_parser.cpp)
  target_link_libraries(test_yaml_parser gtest auto_ros_parameters)
  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils gtest auto_ros_parameters)
  ament_add_gtest(test_parameters test/test_parameters.cpp)
  target_link_libraries(test_parameters gtest auto_ros_parameters)
  ament_add_gtest(test_parameter_client test/test_parameter_client.cpp)
  target_link_libraries(test_parameter_client gtest auto_ros_parameters)

  add_launch_test(
    test/test_server_node_launch.py
    TARGET test_server_node_ok
    ARGS "dir_valid:=OK"
  )
  add_launch_test(
    test/test_server_node_launch.py
    TARGET test_server_node_empty
    ARGS "dir_valid:=Empty"
  )
  add_launch_test(
    test/test_server_node_launch.py
    TARGET test_server_node_not_exists
    ARGS "dir_valid:=NotExists"
  )
  add_launch_test(
    test/test_server_node_launch.py
    TARGET test_server_node_invalid
    ARGS "dir_valid:=Invalid"
  )

  add_launch_test(test/test_parameter_server_run.py)
  add_launch_test(test/test_parameter_server_launch.py)
  add_launch_test(test/examples/test_parameters_usage.py)
  add_launch_test(test/examples/test_yaml_parser_usage.py)
  add_launch_test(test/examples/test_parameter_client.py)
endif()

install(
  DIRECTORY include/ros2_uav_parameters/
  DESTINATION include
)

install(
  TARGETS auto_ros_parameters parameter_server
  EXPORT export_auto_ros_parameters
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS parameters_usage yaml_parser_usage parameter_client
  RUNTIME DESTINATION share/${PROJECT_NAME}/examples
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_auto_ros_parameters HAS_LIBRARY_TARGET)
ament_package()
