cmake_minimum_required(VERSION 3.8)
project(ros2_uav_parameters)

set(CMAKE_CXX_STANDARD 23)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_cpp REQUIRED)
find_package(ros2_uav_interfaces REQUIRED)
find_package(ros2_uav_cpp REQUIRED)

add_library(parameter_client SHARED src/parameters/parameter.cpp src/parameter_client.cpp)
ament_target_dependencies(parameter_client rclcpp ros2_uav_interfaces)
target_link_libraries(parameter_client uav_cpp::parameters)
target_include_directories(parameter_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(parameter_server src/parameter_server.cpp)
ament_target_dependencies(parameter_server rclcpp)
target_link_libraries(parameter_server parameter_client ros2_uav_cpp::ros2_uav_logger)

add_executable(parameter_client_usage examples/parameter_client_usage.cpp)
target_link_libraries(parameter_client_usage parameter_client)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(parameter_test test/parameter_test.cpp)
  target_link_libraries(parameter_test parameter_client)

  ament_add_gtest(parameter_server_test test/parameter_server_test.cpp)
  target_link_libraries(parameter_server_test parameter_client)

  ament_add_gtest(parameter_client_test test/parameter_client_test.cpp)
  target_link_libraries(parameter_client_test parameter_client)

  add_launch_test(
    test/server_node_launch_test.py
    TARGET test_server_node_ok
    ARGS "dir_valid:=OK"
  )
  add_launch_test(
    test/server_node_launch_test.py
    TARGET test_server_node_empty
    ARGS "dir_valid:=Empty"
  )
  add_launch_test(
    test/server_node_launch_test.py
    TARGET test_server_node_not_exists
    ARGS "dir_valid:=NotExists"
  )
  add_launch_test(
    test/server_node_launch_test.py
    TARGET test_server_node_invalid
    ARGS "dir_valid:=Invalid"
  )
  add_launch_test(test/parameter_server_run_test.py)
  add_launch_test(test/parameter_server_launch_test.py)
  add_launch_test(test/examples/test_parameter_client_usage.py)
endif()

install(
  DIRECTORY include/ros2_uav_parameters/
  DESTINATION include
)

install(
  TARGETS parameter_server parameter_client
  EXPORT export_parameter_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS parameter_client_usage
  RUNTIME DESTINATION share/${PROJECT_NAME}/examples
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_parameter_client HAS_LIBRARY_TARGET)
ament_package()
