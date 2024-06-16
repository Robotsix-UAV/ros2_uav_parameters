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
#include <ros2_uav_interfaces/srv/parameter_client_register.hpp>

namespace uav_ros2::parameters
{
using rcl_interfaces::msg::ParameterDescriptor;

/**
 * @class Parameter
 * @brief A class for managing ROS2 parameters with callback functionality.
 *
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
   * @param node A pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param name The name of the parameter.
   * @param value The initial value of the parameter.
   * @param descriptor The descriptor of the parameter.
   */
  template<typename T>
  Parameter(
    rclcpp::Node * node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const T & value, const ParameterDescriptor & descriptor)
  : node_(node), param_name_(name)
  {
    node_->declare_parameter(name, value, descriptor);
    cb_handle_ = param_subscriber->add_parameter_callback(
      param_name_, [this](const rclcpp::Parameter & parameter)
      {return this->parameterCallback(parameter);});
  }

  /**
   * @brief Overloaded constructor to initialize and declare a ROS2 parameter with a shared pointer to the node.
   *
   * @tparam T The data type of the parameter value.
   * @param node A shared pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param name The name of the parameter.
   * @param value The initial value of the parameter.
   * @param descriptor The descriptor of the parameter.
   */
  template<typename T>
  Parameter(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const T & value, const ParameterDescriptor & descriptor)
  : Parameter(node.get(), param_subscriber, name, value, descriptor)
  {}

  /**
   * @brief Constructor to initialize and declare a ROS2 parameter from an existing parameter.
   *
   * @param node A pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param parameter The existing parameter.
   */
  Parameter(
    rclcpp::Node * node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const rclcpp::Parameter & parameter);

  /**
   * @brief Overloaded constructor to initialize and declare a ROS2 parameter from an existing parameter with a shared pointer to the node.
   *
   * @param node A shared pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param parameter The existing parameter.
   */
  Parameter(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const rclcpp::Parameter & parameter)
  : Parameter(node.get(), param_subscriber, parameter)
  {}

protected:
  /**
   * @brief Virtual method called when the parameter changes.
   *
   * This method can be overridden in derived classes to handle parameter changes.
   *
   * @param parameter The updated parameter.
   */
  virtual void onParameterChange([[maybe_unused]] const rclcpp::Parameter & parameter);

  /**
   * @brief Callback method for parameter changes.
   *
   * This method is called when the parameter changes and triggers the onParameterChange method.
   *
   * @param parameter The updated parameter.
   */
  virtual void parameterCallback(const rclcpp::Parameter & parameter);

  /**
   * @brief Logs the parameter change.
   *
   * This method logs the details of the parameter change, including the new value of the parameter.
   * The logging format depends on the parameter type.
   *
   * @param parameter The updated parameter.
   */
  void logChange(const rclcpp::Parameter & parameter);

  rclcpp::Node * node_;  ///< Pointer to the ROS2 node.

protected:
  std::string param_name_;  ///< Name of the parameter.

private:
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_; ///< Handle for the parameter callback.
};

/**
 * @class ServerParameter
 * @brief A class for managing ROS2 server parameters with client registration functionality.
 *
 * This class extends the Parameter class to include functionality for registering and unregistering client nodes.
 */
class ServerParameter : public Parameter
{
public:
  /**
   * @brief Constructor to initialize and declare a ROS2 parameter.
   *
   * This constructor declares a parameter with the given name, value, and descriptor,
   * and sets up a callback to handle parameter changes.
   *
   * @tparam T The data type of the parameter value.
   * @param node A pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param name The name of the parameter.
   * @param value The initial value of the parameter.
   * @param descriptor The descriptor of the parameter.
   */
  template<typename T>
  ServerParameter(
    rclcpp::Node * node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const T & value, const ParameterDescriptor & descriptor)
  : Parameter(node, param_subscriber, name, value, descriptor)
  {
    // Replace dots with slashes in the parameter name
    std::string service_name = "param/" + name;
    std::replace(service_name.begin(), service_name.end(), '.', '/');
    service_name += "/register";

    // Create a service to register a new client node
    register_service_ = node->create_service<ros2_uav_interfaces::srv::ParameterClientRegister>(
      service_name,
      std::bind(
        &ServerParameter::handleClientRegistration, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  /**
   * @brief Overloaded constructor to initialize and declare a ROS2 parameter with a shared pointer to the node.
   *
   * @tparam T The data type of the parameter value.
   * @param node A shared pointer to the ROS2 node.
   * @param param_subscriber A shared pointer to the parameter event handler.
   * @param name The name of the parameter.
   * @param value The initial value of the parameter.
   * @param descriptor The descriptor of the parameter.
   */
  template<typename T>
  ServerParameter(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const T & value, const ParameterDescriptor & descriptor)
  : ServerParameter(node.get(), param_subscriber, name, value, descriptor)
  {}

protected:
  /**
   * @brief Virtual method called when the parameter changes.
   *
   * This method can be overridden in derived classes to handle parameter changes.
   *
   * @param parameter The updated parameter.
   */
  void onParameterChange([[maybe_unused]] const rclcpp::Parameter & parameter) override;

private:
  /**
   * @brief Callback method for parameter changes.
   *
   * This method is called when the parameter changes and triggers the onParameterChange method.
   *
   * @param parameter The updated parameter.
   */
  void parameterCallback(const rclcpp::Parameter & parameter) override;

  /**
   * @brief Handles client registration and unregistration requests.
   *
   * This method processes client registration and unregistration requests, updating the list of registered clients accordingly.
   *
   * @param request The client registration request.
   * @param response The client registration response.
   */
  void handleClientRegistration(
    const std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Request> request,
    std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Response> response);

  std::vector<std::string> client_nodes_;  ///< List of registered client nodes.
  std::shared_ptr<rclcpp::Service<ros2_uav_interfaces::srv::ParameterClientRegister>>
  register_service_;                                                                                      ///< Service for registering/unregistering client nodes.
};

}  // namespace uav_ros2::parameters
