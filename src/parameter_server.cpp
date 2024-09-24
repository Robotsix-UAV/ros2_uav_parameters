// Copyright 2024 The Technology Innovation Institute (TII)
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

/**
 * @author Damien SIX (damien@robotsix.net)
 */

#include <filesystem>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <uav_cpp/parameters/yaml_parameter_parser.hpp>
#include "ros2_uav_parameters/parameters/parameter.hpp"

namespace fs = std::filesystem;
using uav_cpp::parameters::YamlParameterParser;
using ros2_uav::parameters::ServerParameter;

class ServerNode : public rclcpp::Node, public uav_cpp::logger::LogTagHolder
{
public:
  ServerNode()
  : Node("parameter_server"), LogTagHolder("Parameter Server")
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
      if (parameters_.find(name) != parameters_.end()) {
        throw std::runtime_error(
                "Parameter " + name +
                " was found twice in the configuration files");
      }
      UAVCPP_DEBUG_TAG(this, "Adding parameter {}", name);
      parameter->createRegisterService(this);
      parameter->createRosCallback(this, param_subscriber_);
      parameters_.emplace(name, parameter);
    }
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
          UAVCPP_ERROR_TAG(
            this,
            "Error parsing file {}: {}",
            entry.path().string().c_str(), e.what());
          throw e;
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

  uav_cpp::logger::LogManager::getInstance("parameter_server.log");

  auto node = std::make_shared<ServerNode>();

  auto default_config_folder = ament_index_cpp::get_package_share_directory("ros2_uav_parameters");
  default_config_folder += "/config";
  auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  param_descriptor.description = "Folder containing YAML configuration files";
  node->declare_parameter("config_directory", default_config_folder, param_descriptor);

  // Need to spin to get the parameter necessary to load all the others
  rclcpp::spin_some(node);

  node->loadParametersFromDirectory();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
