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

#include "auto_ros_parameters/parameter_client.hpp"
#include <algorithm>

namespace ros2_uav::parameters
{

ParameterClient::ParameterClient(
  const std::string & node_name, std::vector<std::string> & required_parameters,
  const std::string & server_name)
: Node(node_name), server_name_(server_name)
{
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, server_name);
  RCLCPP_INFO(get_logger(), "Waiting for parameter server to start");
  if (!parameter_client->wait_for_service(1s)) {
    throw std::runtime_error("Parameter server " + server_name + " not available");
  }
  // Wait for the parameter server to load the parameters
  std::this_thread::sleep_for(1s);
  for (const auto & parameter : required_parameters) {
    if (!parameter_client->has_parameter(parameter)) {
      throw rclcpp::exceptions::InvalidParametersException(
              "Required parameter " + parameter + " is missing");
    }
  }
  auto rclcpp_remote_parameters = parameter_client->get_parameters(
    required_parameters, 1s);
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  for (const auto & parameter : rclcpp_remote_parameters) {
    remote_parameters_.push_back(
      std::make_shared<ros2_uav::parameters::Parameter>(
        this, param_subscriber_,
        parameter));
  }
  register_thread_ = std::jthread([this]() {registerParameters();});
}

ParameterClient::~ParameterClient()
{
  if (register_thread_.joinable()) {
    register_thread_.join();
  }
  registerParameters(false);
}

void ParameterClient::registerParameters(bool register_client)
{
  for (auto & parameter : remote_parameters_) {
    std::string service_name = parameter->getName();
    std::replace(service_name.begin(), service_name.end(), '.', '/');
    service_name = server_name_ + "/param/" + service_name + "/register";
    auto client_register =
      this->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
      service_name);
    auto request =
      std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
    request->client_name = this->get_fully_qualified_name();
    request->register_client = register_client;
    if (!client_register->wait_for_service(1s)) {
      throw std::runtime_error(
              "Failed to connect to parameter server service for parameter " +
              parameter->getName());
    }
    if (register_client) {
      RCLCPP_INFO(
        get_logger(), "Registering parameter %s with service %s",
        parameter->getName().c_str(), service_name.c_str());
      auto future = client_register->async_send_request(request);
      future.wait_for(1s);
      auto result = future.get();
      if (!result->success) {
        throw std::runtime_error(
                "Failed to register client in parameter server");
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "Unregistering parameter %s with service %s",
        parameter->getName().c_str(), service_name.c_str());
      client_register->async_send_request(request);
    }
  }
}

}  // namespace ros2_uav::parameters