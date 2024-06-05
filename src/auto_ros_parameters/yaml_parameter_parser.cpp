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

#include "auto_ros_parameters/yaml_parameter_parser.hpp"
#include <fstream>
#include <limits>
#include <stdexcept>
#include <sstream>

namespace uav_ros2
{
using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::IntegerRange;
using rcl_interfaces::msg::ParameterDescriptor;

YamlParameterParser::YamlParameterParser(const std::string & file_path)
{
  parseYamlFile(file_path);
}

const std::vector<std::tuple<std::string, YamlParameterParser::ParameterValue,
  ParameterDescriptor>> & YamlParameterParser::getParameters() const
{
  return parameters_;
}

void YamlParameterParser::parseYamlFile(const std::string & file_path)
{
  YAML::Node config = YAML::LoadFile(file_path);
  if (!config.IsMap()) {
    throw std::runtime_error("Invalid YAML file. Top level must be a map.");
  }
  for (const auto & group : config) {
    if (!group.second.IsMap()) {
      throw std::runtime_error("Invalid YAML file. Parameter groups must be maps.");
    }
    std::string group_name = group.first.as<std::string>();
    for (const auto & param : group.second) {
      if (!param.second.IsMap()) {
        throw std::runtime_error("Invalid YAML file. Parameters must be maps.");
      }

      std::string param_name = param.first.as<std::string>();
      std::string full_param_name = group_name + "." + param_name;
      auto param_node = param.second;

      if (!param_node["default"] || !param_node["type"]) {
        std::stringstream ss;
        ss << "Parameter " << full_param_name << " is missing required 'default' or 'type' field.";
        throw std::runtime_error(ss.str());
      }

      ParameterDescriptor descriptor;
      descriptor.description =
        param_node["description"] ? param_node["description"].as<std::string>() : "";
      if (param_node["constraints_description"]) {
        descriptor.additional_constraints =
          param_node["constraints_description"].as<std::string>();
      }

      std::string type = param_node["type"].as<std::string>();

      if (param_node["default"].IsSequence()) {
        // Handle arrays
        if (type == "int") {
          std::vector<int> default_value = param_node["default"].as<std::vector<int>>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "double") {
          std::vector<double> default_value = param_node["default"].as<std::vector<double>>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "bool") {
          std::vector<bool> default_value = param_node["default"].as<std::vector<bool>>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "string") {
          std::vector<std::string> default_value =
            param_node["default"].as<std::vector<std::string>>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else {
          std::stringstream ss;
          ss << "Parameter " << full_param_name << " has an invalid type: " << type;
          throw std::runtime_error(ss.str());
        }
      } else if (param_node["default"].IsScalar()) {
        // Handle scalars
        if (type == "int") {
          int default_value = param_node["default"].as<int>();
          if (param_node["min"] || param_node["max"]) {
            IntegerRange range;
            range.from_value =
              param_node["min"] ? param_node["min"].as<int>() : std::numeric_limits<int>::min();
            range.to_value =
              param_node["max"] ? param_node["max"].as<int>() : std::numeric_limits<int>::max();
            descriptor.set__integer_range({range});
          }
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "double") {
          double default_value = param_node["default"].as<double>();
          if (param_node["min"] || param_node["max"]) {
            FloatingPointRange range;
            range.from_value =
              param_node["min"] ? param_node["min"].as<double>() : -std::numeric_limits<double>::
              infinity();
            range.to_value =
              param_node["max"] ? param_node["max"].as<double>() : std::numeric_limits<double>::
              infinity();
            descriptor.set__floating_point_range({range});
          }
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "bool") {
          bool default_value = param_node["default"].as<bool>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else if (type == "string") {
          std::string default_value = param_node["default"].as<std::string>();
          parameters_.emplace_back(full_param_name, default_value, descriptor);
        } else {
          std::stringstream ss;
          ss << "Parameter " << full_param_name << " has an invalid type: " << type;
          throw std::runtime_error(ss.str());
        }
      } else {
        throw std::runtime_error(
                "Invalid YAML file. Parameter default value must be a scalar or a sequence.");
      }
    }
  }
}
}  // namespace uav_ros2
