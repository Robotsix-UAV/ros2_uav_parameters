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

#include <algorithm>
#include "ros2_uav_parameters/parameter_client.hpp"

namespace ros2_uav::parameters
{
using std::chrono_literals::operator""s;

ParameterClient::ParameterClient(
  const std::string & node_name, const std::vector<std::string> & required_parameters,
  const std::string & server_name)
: Node(node_name), server_name_(server_name)
{
  if (server_name_ == "") {
    if (std::string(get_namespace()) == "/") {
      server_name_ = "parameter_server";
    } else {
      server_name_ = std::string(get_namespace()) + "/parameter_server";
    }
  }
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, server_name_);
  RCLCPP_INFO(get_logger(), "Waiting for parameter server %s to start", server_name_.c_str());
  if (!parameter_client->wait_for_service(5s)) {
    throw std::runtime_error("Parameter server " + server_name_ + " not available");
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
    remote_parameters_.emplace(
      parameter.get_name(),
      std::make_shared<ros2_uav::parameters::Parameter>(parameter));
  }
  register_thread_ = std::jthread([this]() {registerParameters();});
}

ParameterClient::~ParameterClient()
{
  if (register_thread_.joinable()) {
    register_thread_.join();
  }
}

void ParameterClient::registerParameters()
{
  for (auto & [name, parameter] : remote_parameters_) {
    auto ros_param = std::static_pointer_cast<ros2_uav::parameters::Parameter>(parameter);
    ros_param->createRosCallback(this, param_subscriber_);
    std::string service_name = parameter->getName();
    std::replace(service_name.begin(), service_name.end(), '.', '/');
    service_name = server_name_ + "/param/" + service_name + "/register";
    auto client_register =
      this->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
      service_name);
    auto request =
      std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
    request->client_name = this->get_fully_qualified_name();
    request->register_client = true;
    if (!client_register->wait_for_service(1s)) {
      // LCOV_EXCL_START
      // Tested with death test not captured by gcov
      throw std::runtime_error(
              "Failed to connect to parameter server service for parameter " +
              parameter->getName());
      // LCOV_EXCL_STOP
    }
    RCLCPP_INFO(
      get_logger(), "Registering parameter %s with service %s",
      parameter->getName().c_str(), service_name.c_str());
    auto future = client_register->async_send_request(request);
    future.wait_for(1s);
    auto result = future.get();
    if (!result->success) {
      // LCOV_EXCL_START
      // Tested with death test not captured by gcov
      throw std::runtime_error(
              "Failed to register client in parameter server");
      // LCOV_EXCL_STOP
    }
  }
}

}  // namespace ros2_uav::parameters
