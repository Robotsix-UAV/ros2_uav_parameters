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

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "auto_ros_parameters/parameters.hpp"

namespace uav_ros2
{
using std::chrono_literals::operator""s;

class ParameterClient : public rclcpp::Node
{
public:
  ParameterClient(
    const std::string & node_name, std::vector<std::string> & required_parameters,
    const std::string & server_name = "parameter_server")
  : Node(node_name)
  {
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, server_name);
    while (!parameter_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Interrupted while waiting for the parameter server");
      }
      RCLCPP_INFO(get_logger(), "Waiting for parameter server to start");
    }
    for (const auto & parameter : required_parameters) {
      if (!parameter_client->has_parameter(parameter)) {
        throw rclcpp::exceptions::InvalidParametersException(
                "Required parameter " + parameter + "is missing");
      }
    }
    std::vector<rclcpp::Parameter> remote_parameters_ = parameter_client->get_parameters(
      required_parameters, 1s);
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    for (const auto & parameter : remote_parameters_) {
      remote_parameters_.push_back(
        uav_ros2::parameters::Parameter(
          this, param_subscriber_,
          parameter));
      registerParameters();
    }
  }

  ~ParameterClient()
  {
    unregisterParameters();
  }

  void registerParameters()
  {
    for (auto & parameter : remote_parameters_) {
      auto client_register = node->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
        server_name + "/param" + parameter.getName() + "/register");
      auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
      request->client_name = this->get_fully_qualified_name();
      request->register_client = true;
      auto result = client_register->sync_send_request(request);
      if (!result->success) {
        unregisterParameters();
        throw std::runtime_error("Failed to register parameter " + parameter.getName());
      }
    }
  }

  void unregisterParameters()
  {
    for (auto & parameter : remote_parameters_) {
      auto client_register = node->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
        server_name + "/param" + parameter.getName() + "/register");
      auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
      request->client_name = this->get_fully_qualified_name();
      request->register_client = false;
      auto result = client_register->async_send_request(request);
    }
  }

private:
  std::vector<uav_ros2::parameters::Parameter> remote_parameters_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
};
}  // namespace uav_ros2
