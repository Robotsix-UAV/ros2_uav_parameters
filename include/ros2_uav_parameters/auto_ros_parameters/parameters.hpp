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

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace uav_ros2
{
using rcl_interfaces::msg::ParameterDescriptor;

/**
 * @class Parameter
 * @brief A class for managing ROS2 parameters with callback functionality.
 *
 * THIS IS A TEST
 * This class encapsulates the functionality for declaring and managing ROS2 parameters.
 * It allows for the automatic invocation of callbacks when parameter values change.
 */
class Parameter
{
public:
  /**
   * @brief Constructor to initialize and declare a ROS2 parameter.
   *
   * This constructor declares a parameter with the given name, value, and descriptor,
   * and sets up a callback to handle parameter changes.
   *
   * @tparam T The data type of the parameter value.
   * @param node A shared pointer to the ROS2 node.
   * @param param_suscriber A shared pointer to the parameter event handler.
   * @param name The name of the parameter.
   * @param value The initial value of the parameter.
   * @param descriptor The descriptor of the parameter.
   */
  template<typename T>
  Parameter(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_suscriber,
    const std::string & name, const T & value, const ParameterDescriptor & descriptor)
  :     param_name_(name), node_(node)
  {
    node_->declare_parameter(name, value, descriptor);
    cb_handle_ = param_suscriber->add_parameter_callback(
      param_name_, [this](const rclcpp::Parameter & parameter)
      {return this->parameterCallback(parameter);});
  }

protected:
  /**
   * @brief Virtual method called when the parameter changes.
   *
   * This method can be overridden in derived classes to handle parameter changes.
   *
   * @param parameter The updated parameter.
   */
  virtual void onParameterChange([[maybe_unused]] const rclcpp::Parameter & parameter);

private:
  /**
   * @brief Callback method for parameter changes.
   *
   * This method is called when the parameter changes and triggers the onParameterChange method.
   *
   * @param parameter The updated parameter.
   */
  void parameterCallback(const rclcpp::Parameter & parameter);

  /**
   * @brief Logs the parameter change.
   *
   * This method logs the details of the parameter change, including the new value of the parameter.
   * The logging format depends on the parameter type.
   *
   * @param parameter The updated parameter.
   */
  void logChange(const rclcpp::Parameter & parameter);

  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  ///< Handle for the parameter callback.
  std::string param_name_;  ///< Name of the parameter.

protected:
  rclcpp::Node::SharedPtr node_;  ///< Shared pointer to the ROS2 node.
};

}  // namespace uav_ros2
