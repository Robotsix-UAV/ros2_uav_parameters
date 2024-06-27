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
#include <uav_cpp/parameters/yaml_parameter_parser.hpp>
#include <ros2_uav_cpp/ros2_logger.hpp>
#include "ros2_uav_parameters/parameters/parameter.hpp"

namespace fs = std::filesystem;
using uav_cpp::parameters::YamlParameterParser;
using ros2_uav::parameters::ServerParameter;
using ros2_uav::utils::RosLoggerInterface;

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

  void initParameters(const YamlParameterParser<ServerParameter> & parser)
  {
    auto new_parameters = parser.getParameters();
    for (auto & [name, parameter] : new_parameters) {
      parameter->createRegisterService(this);
      parameter->createRosCallback(this, param_subscriber_);
    }
    parameters_.insert(new_parameters.begin(), new_parameters.end());
    //TODO(robotsix): Proper error for double parameters
  }

  void loadParametersFromDirectory()
  {
    auto config_directory = get_parameter("config_directory").as_string();
    for (const auto & entry : fs::recursive_directory_iterator(config_directory)) {
      if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
        try {
          YamlParameterParser<ServerParameter> parser(entry.path().string());
          initParameters(parser);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            get_logger(), "Error parsing file %s: %s",
            entry.path().string().c_str(), e.what());
        }
      }
    }
  }
  std::map<std::string, std::shared_ptr<ServerParameter>> parameters_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServerNode>();

  auto default_config_folder = ament_index_cpp::get_package_share_directory("ros2_uav_parameters");
  default_config_folder += "/config";
  auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  param_descriptor.description = "Folder containing YAML configuration files";
  node->declare_parameter("config_directory", default_config_folder, param_descriptor);

  // Set the logger to node logger for the uav_cpp library
  auto logger = std::make_shared<RosLoggerInterface>(node->get_logger());
  uav_cpp::logger::Logger::setCustomLogger(logger);

  // Need to spin to get the parameter necessary to load all the others
  rclcpp::spin_some(node);

  node->loadParametersFromDirectory();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
