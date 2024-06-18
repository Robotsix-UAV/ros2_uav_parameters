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

using std::chrono_literals::operator""s;

namespace ros2_uav::parameters
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

void ServerParameter::parameterCallback(const rclcpp::Parameter & parameter)
{
  onParameterChange(parameter);
  logChange(parameter);

  std::vector<std::string> invalid_clients;
  for (const auto & client : client_nodes_) {
    auto request_client = client.second;
    // Send the parameter to the client
    try {
      if (!request_client->wait_for_service(1s)) {
        throw std::runtime_error("Service not available");
      }
      auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      auto parameter_msg = parameter.to_parameter_msg();
      request->parameters.push_back(parameter_msg);
      request_client->async_send_request(request);
      RCLCPP_INFO(
        node_->get_logger(), "Sending parameter %s to client %s",
        parameter.get_name().c_str(), client.first.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to send parameter %s to client %s: %s",
        parameter.get_name().c_str(), client.first.c_str(), e.what());
      RCLCPP_WARN(
        node_->get_logger(), "Unregistering client node %s for parameter %s",
        client.first.c_str(), param_name_.c_str());
      invalid_clients.push_back(client.first);
    }
  }
  // Unregister invalid clients
  for (const auto & client : invalid_clients) {
    auto request_client = std::find_if(
      client_nodes_.begin(), client_nodes_.end(),
      [&client](auto & client_node) {return client_node.first == client;});
    if (request_client != client_nodes_.end()) {
      client_nodes_.erase(request_client);
    }
  }
}

void ServerParameter::onParameterChange([[maybe_unused]] const rclcpp::Parameter & parameter)
{
}

void ServerParameter::handleClientRegistration(
  const std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Request> request,
  std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Response> response)
{
  auto client_node_name = request->client_name;
  if (request->register_client) {
    RCLCPP_INFO(
      node_->get_logger(), "Registering client node %s for parameter %s",
      client_node_name.c_str(), param_name_.c_str());
    // Check if the client node is already registered
    if (std::find_if(
        client_nodes_.begin(), client_nodes_.end(),
        [&client_node_name](auto & client) {return client.first == client_node_name;}) !=
      client_nodes_.end())
    {
      RCLCPP_WARN(
        node_->get_logger(), "Client node %s is already registered for parameter %s",
        client_node_name.c_str(), param_name_.c_str());
      response->success = true;
      response->message = "Client node is already registered";
      return;
    }
    auto client_set_parameters = node_->create_client<rcl_interfaces::srv::SetParameters>(
      client_node_name + "/set_parameters");
    response->success = true;
    client_nodes_.push_back({client_node_name, client_set_parameters});
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Unregistering client node %s for parameter %s",
      client_node_name.c_str(), param_name_.c_str());
    auto it = std::find_if(
      client_nodes_.begin(), client_nodes_.end(),
      [&client_node_name](auto & client) {return client.first == client_node_name;});
    if (it != client_nodes_.end()) {
      client_nodes_.erase(it);
      response->success = true;
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Client node %s is not registered for parameter %s",
        client_node_name.c_str(), param_name_.c_str());
      response->success = false;
      response->message = "Client node is not registered";
    }
  }
}

}  // namespace ros2_uav::parameters
