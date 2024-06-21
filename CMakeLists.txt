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

add_library(auto_parameters SHARED src/parameters/parameter.cpp)
ament_target_dependencies(auto_parameters rclcpp ros2_uav_interfaces)
target_link_libraries(auto_parameters uav_cpp::parameters)
target_include_directories(auto_parameters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(parameter_test test/parameter_test.cpp)
  target_link_libraries(parameter_test auto_parameters)
endif()

# install(
#   DIRECTORY include/ros2_uav_parameters/
#   DESTINATION include
# )

# install(
#   TARGETS parameter_server parameter_client
#   EXPORT export_parmeter_client
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   INCLUDES DESTINATION include
# )

# install(
#   TARGETS parameter_client_usage
#   RUNTIME DESTINATION share/${PROJECT_NAME}/examples
# )

# install(
#   DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
