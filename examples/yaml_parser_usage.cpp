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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <variant>
#include <filesystem>
// START_EXAMPLE yaml_parser_usage
#include "auto_ros_parameters/yaml_parameter_parser.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

int main()
{
// END_EXAMPLE yaml_parser_usage
  // Sample YAML content
  std::string yaml_content =
    R"(
group1:
  parameter1:
    default: 10
    type: int
    description: "An integer parameter"
  parameter2:
    default: "hello"
    type: string
    description: "A string parameter"
  )";

  // Create a temporary file
  std::filesystem::path temp_path = std::filesystem::temp_directory_path() / "temp_parameters.yaml";
  std::ofstream temp_file(temp_path);
  temp_file << yaml_content;
  temp_file.close();

// START_EXAMPLE yaml_parser_usage
  // Parse parameters from the temporary YAML file
  ros2_uav::parameters::YamlParameterParser parser(temp_path.string());
  auto parameters = parser.getParameters();

  // Iterate over the parameters
  for (const auto & param : parameters) {
    std::string name;
    // Use std::variant to store different types of parameter values
    ros2_uav::parameters::YamlParameterParser::ParameterValue value;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    std::tie(name, value, descriptor) = param;

    // Display the parameter name
    std::cout << "Parameter name: " << name << std::endl;
  }
// END_EXAMPLE yaml_parser_usage

  // Remove the temporary file
  std::filesystem::remove(temp_path);

// START_EXAMPLE yaml_parser_usage
  return 0;
}
// END_EXAMPLE yaml_parser_usage
