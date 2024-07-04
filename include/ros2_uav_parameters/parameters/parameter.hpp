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

#include <functional>
#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ros2_uav_interfaces/srv/parameter_client_register.hpp>
#include <uav_cpp/parameters/parameter.hpp>

namespace ros2_uav::parameters
{
using uav_cpp::utils::SmartPointerMixin;
using rcl_interfaces::msg::ParameterDescriptor;
using ros2_uav_interfaces::srv::ParameterClientRegister;
using rcl_interfaces::srv::SetParameters;

/**
 * @class Parameter
 * @brief A class wrapping uav_cpp::parameters::Parameter with ROS2 parameter callback functionality.
 */
class Parameter : public SmartPointerMixin<Parameter, uav_cpp::parameters::Parameter>
{
public:
  using SmartPointerMixin<Parameter, uav_cpp::parameters::Parameter>::SmartPointerMixin;

  /**
   * @brief Constructor to initialize a Parameter object with a given node, parameter subscriber, name, value, and description.
   *
   * @param node Pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   * @param name Name of the parameter.
   * @param value Value of the parameter.
   * @param description Description of the parameter (optional).
   */
  Parameter(
    rclcpp::Node * node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const uav_cpp::parameters::ParameterType & value,
    const std::string & description = "");

  /**
   * @brief Constructor to initialize a Parameter object with a shared pointer to the node, parameter subscriber, name, value, and description.
   *
   * @param node Shared pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   * @param name Name of the parameter.
   * @param value Value of the parameter.
   * @param description Description of the parameter (optional).
   */
  Parameter(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const uav_cpp::parameters::ParameterType & value,
    const std::string & description = "");

  /**
   * @brief Constructor to initialize from an existing ROS2 parameter. Requires calling createRosCallback to enable parameter callback.
   *
   * @param parameter The existing parameter.
   */
  explicit Parameter(const rclcpp::Parameter & parameter);

  /**
   * @brief Creates a ROS parameter callback for the given node and parameter subscriber.
   *
   * @param node Pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   */
  void createRosCallback(
    rclcpp::Node * node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber);

  /**
   * @brief Creates a ROS parameter callback for the given shared pointer to the node and parameter subscriber.
   *
   * @param node Shared pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   */
  void createRosCallback(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber);

protected:
  /**
   * @brief Creates a parameter callback function given its type.
   *
   * @return A function that handles parameter changes.
   */
  std::function<void(const rclcpp::Parameter &)> createParameterCallback();

  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_ = nullptr;
  ///< Handle for the parameter callback.
};

/**
 * @class ServerParameter
 * @brief A class for managing ROS2 server parameters with client registration functionality.
 *
 * This class extends the Parameter class to include functionality for registering and unregistering client nodes.
 * Registered client nodes receive updates when the parameter changes.
 */
class ServerParameter : public Parameter
{
public:
  using Parameter::Parameter;

  /**
   * @brief Constructor to initialize a ServerParameter object with a given node, parameter subscriber, name, value, and description.
   *
   * @param node Pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   * @param name Name of the parameter.
   * @param value Value of the parameter.
   * @param description Description of the parameter (optional).
   */
  ServerParameter(
    rclcpp::Node * node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const uav_cpp::parameters::ParameterType & value,
    const std::string & description = "");

  /**
   * @brief Constructor to initialize a ServerParameter object with a shared pointer to the node, parameter subscriber, name, value, and description.
   *
   * @param node Shared pointer to the ROS2 node.
   * @param param_subscriber Shared pointer to the parameter event handler.
   * @param name Name of the parameter.
   * @param value Value of the parameter.
   * @param description Description of the parameter (optional).
   */
  ServerParameter(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
    const std::string & name, const uav_cpp::parameters::ParameterType & value,
    const std::string & description = "");

  /**
   * @brief Creates a service for registering and unregistering client nodes.
   *
   * @param node Pointer to the ROS2 node.
   */
  void createRegisterService(rclcpp::Node * node);

  /**
   * @brief Creates a service for registering and unregistering client nodes.
   *
   * @param node Shared pointer to the ROS2 node.
   */
  void createRegisterService(rclcpp::Node::SharedPtr node);

private:
  /**
   * @brief Handles parameter changes and notifies registered clients.
   */
  void onParameterChange();

  /**
   * @brief Handles client registration and unregistration requests.
   *
   * This method processes client registration and unregistration requests, updating the list of registered clients accordingly.
   *
   * @param request The client registration request.
   * @param response The client registration response.
   */
  void handleClientRegistration(
    const std::shared_ptr<ParameterClientRegister::Request> request,
    std::shared_ptr<ParameterClientRegister::Response> response);

  std::vector<std::pair<std::string,
    std::shared_ptr<rclcpp::Client<SetParameters>>>> client_nodes_;
  ///< List of registered client nodes.

  std::shared_ptr<rclcpp::Service<ParameterClientRegister>>
  register_service_;  ///< Service for registering/unregistering client nodes.
  rclcpp::Node * node_ = nullptr;  ///< Pointer to the ROS2 node.
};

}  // namespace ros2_uav::parameters
