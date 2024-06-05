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

#pragma once

#include <yaml-cpp/yaml.h>
#include <vector>
#include <tuple>
#include <string>
#include <variant>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// TODO(Damien SIX): Min and max keys are not handled for sequences.
// Add support for this if possible.

namespace uav_ros2
{
/**
 * @class YamlParameterParser
 * @brief A class to parse parameters from a YAML file.
 */
class YamlParameterParser
{
public:
  /**
   * @brief Type alias for parameter value types.
   *
   * ParameterValue can be an int, double, string, bool, or vectors of these types.
   */
  using ParameterValue = std::variant<int, double, std::string, bool, std::vector<int>,
      std::vector<double>, std::vector<std::string>, std::vector<bool>>;

  /**
   * @brief Constructor that initializes the parser with a YAML file path.
   *
   * @param file_path The path to the YAML file containing parameters.
   */
  explicit YamlParameterParser(const std::string & file_path);

  /**
   * @brief Gets the parsed parameters.
   *
   * @return A constant reference to a vector of tuples containing parameter names,
   * values, and their descriptors.
   */
  const std::vector<std::tuple<std::string, ParameterValue,
    rcl_interfaces::msg::ParameterDescriptor>> & getParameters() const;

private:
  /**
   * @brief Parses the YAML file to extract parameters.
   *
   * This method reads the YAML file specified by file_path and extracts the parameters
   * along with their values and descriptors.
   *
   * @param file_path The path to the YAML file.
   */
  void parseYamlFile(const std::string & file_path);

  std::vector<std::tuple<std::string, ParameterValue,
    rcl_interfaces::msg::ParameterDescriptor>> parameters_;  ///< Parsed parameters.
};

}  // namespace uav_ros2
