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

#include "auto_ros_parameters/parameters.hpp"
#include "auto_ros_parameters/utils.hpp"

namespace uav_ros2
{
void Parameter::onParameterChange([[maybe_unused]] const rclcpp::Parameter & parameter)
{
}

void Parameter::parameterCallback(const rclcpp::Parameter & parameter)
{
  onParameterChange(parameter);
  logChange(parameter);
}

void Parameter::logChange(const rclcpp::Parameter & parameter)
{
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      RCLCPP_INFO(
        node_->get_logger(), "Parameter %s set to %s",
        param_name_.c_str(), parameter.as_bool() ? "true" : "false");
      break;

    case rclcpp::ParameterType::PARAMETER_INTEGER:
      RCLCPP_INFO(
        node_->get_logger(), "Parameter %s set to %ld",
        param_name_.c_str(), parameter.as_int());
      break;

    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      RCLCPP_INFO(
        node_->get_logger(), "Parameter %s set to %s",
        param_name_.c_str(), utils::doubleToStringTrimZero(parameter.as_double()).c_str());
      break;

    case rclcpp::ParameterType::PARAMETER_STRING:
      RCLCPP_INFO(
        node_->get_logger(), "Parameter %s set to %s",
        param_name_.c_str(), parameter.as_string().c_str());
      break;

    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      {
        bool first = true;
        std::string bool_array = "[";
        for (const auto & value : parameter.as_bool_array()) {
          if (!first) {
            bool_array += ", ";
          }
          first = false;
          bool_array += value ? "true" : "false";
        }
        bool_array += "]";
        RCLCPP_INFO(
          node_->get_logger(), "Parameter %s set to %s",
          param_name_.c_str(), bool_array.c_str());
        break;
      }

    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      {
        std::string int_array = "[";
        bool first = true;
        for (const auto & value : parameter.as_integer_array()) {
          if (!first) {
            int_array += ", ";
          }
          first = false;
          int_array += std::to_string(value);
        }
        int_array += "]";
        RCLCPP_INFO(
          node_->get_logger(), "Parameter %s set to %s",
          param_name_.c_str(), int_array.c_str());
        break;
      }

    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      {
        std::string double_array = "[";
        bool first = true;
        for (const auto & value : parameter.as_double_array()) {
          if (!first) {
            double_array += ", ";
          }
          first = false;
          double_array += utils::doubleToStringTrimZero(value);
        }
        double_array += "]";
        RCLCPP_INFO(
          node_->get_logger(), "Parameter %s set to %s",
          param_name_.c_str(), double_array.c_str());
        break;
      }

    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      {
        std::string string_array = "[";
        bool first = true;
        for (const auto & value : parameter.as_string_array()) {
          if (!first) {
            string_array += ", ";
          }
          first = false;
          string_array += value;
        }
        string_array += "]";
        RCLCPP_INFO(
          node_->get_logger(), "Parameter %s set to %s",
          param_name_.c_str(), string_array.c_str());
        break;
      }

    // LCOV_EXCL_START
    default:
      RCLCPP_INFO(node_->get_logger(), "Parameter %s set", param_name_.c_str());
      break;
      // LCOV_EXCL_STOP
  }
}
}  // namespace uav_ros2
