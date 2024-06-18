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

#include <filesystem>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "auto_ros_parameters/parameters.hpp"
#include "auto_ros_parameters/yaml_parameter_parser.hpp"

namespace fs = std::filesystem;
using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::ParameterDescriptor;
using ros2_uav::parameters::YamlParameterParser;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("parameter_server")
  {
    // Create a parameter event handler
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> getParameterEventHandler() const
  {
    return param_subscriber_;
  }

  void initParameters(const ros2_uav::parameters::YamlParameterParser & parser)
  {
    // Declare parameters from the parser
    for (const auto & param : parser.getParameters()) {
      std::string name;
      YamlParameterParser::ParameterValue value;
      ParameterDescriptor descriptor;
      std::tie(name, value, descriptor) = param;
      addParameter(name, value, descriptor);
    }
  }

  void loadParametersFromDirectory()
  {
    auto config_directory = get_parameter("config_directory").as_string();
    for (const auto & entry : fs::recursive_directory_iterator(config_directory)) {
      if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
        try {
          YamlParameterParser parser(entry.path().string());
          initParameters(parser);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            get_logger(), "Error parsing file %s: %s",
            entry.path().string().c_str(), e.what());
        }
      }
    }
  }

private:
  void addParameter(
    const std::string & name, const YamlParameterParser::ParameterValue & value,
    const ParameterDescriptor & descriptor)
  {
    std::visit(
      [this, &name, &descriptor](auto && val)
      {
        parameters_.push_back(
          std::make_shared<ros2_uav::parameters::ServerParameter>(
            shared_from_this(),
            param_subscriber_, name, val, descriptor));
      },
      value);
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::vector<std::shared_ptr<ros2_uav::parameters::ServerParameter>> parameters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServerNode>();
  auto default_config_folder = ament_index_cpp::get_package_share_directory("ros2_uav_parameters");
  default_config_folder += "/config";
  auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  param_descriptor.description = "Folder containing YAML configuration files";
  auto config_file = ros2_uav::parameters::Parameter(
    node,
    node->getParameterEventHandler(), "config_directory", default_config_folder, param_descriptor);

  // Need to spin to get the parameter necessary to load all the others
  rclcpp::spin_some(node);

  node->loadParametersFromDirectory();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
