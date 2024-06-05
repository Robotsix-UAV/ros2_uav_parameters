// Copyright 2024 Damien SIX (damien@robotsix.net)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// START_EXAMPLE parameters_usage
#include <rclcpp/rclcpp.hpp>
// END_EXAMPLE parameters_usage
// START_EXAMPLE parameters_callback
#include "auto_ros_parameters/parameters.hpp"

// Class that inherits from uav_ros2::Parameter
class MyParameter
  : public uav_ros2::Parameter
{
public:
  MyParameter(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_handler,
    const std::string & name,
    const int & value,
    rcl_interfaces::msg::ParameterDescriptor descriptor)
  : uav_ros2::Parameter(node, parameter_handler, name, value, descriptor)
  {
  }

protected:
  // Custom callback function in the derived class
  void onParameterChange(const rclcpp::Parameter & parameter)
  {
    if (parameter.as_int() == 42) {
      RCLCPP_INFO(
        node_->get_logger(),
        "The answer to the Ultimate Question of Life, the Universe, and Everything!");
    }
  }
};
// END_EXAMPLE parameters_callback

// START_EXAMPLE parameters_usage
int main()
{
  // Initialize the ROS 2 system
  rclcpp::init(0, nullptr);
  // Create a ROS 2 node named "parameters_usage"
  auto node = rclcpp::Node::make_shared("parameters_usage");
  // Create a parameter event handler
  auto parameter_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);
  // Parameter value
  auto value = 42;
  // Parameter descriptor usage is the same as standard ROS 2 parameters
  auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  param_descriptor.description = "So Long, and Thanks for All the Fish!";
  // Create a parameter with a custom callback function
  auto config_file = MyParameter(
    node, parameter_handler,
    "my_custom_parameter", value, param_descriptor);
  // Spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// END_EXAMPLE parameters_usage
