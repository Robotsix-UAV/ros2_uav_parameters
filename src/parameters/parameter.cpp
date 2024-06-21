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

#include "ros2_uav_parameters/parameters/parameter.hpp"

namespace ros2_uav::parameters
{

using std::chrono_literals::operator""s;

Parameter::Parameter(const rclcpp::Parameter & parameter)
{
  name_ = parameter.get_name();
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    value_ = parameter.get_value<int64_t>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    value_ = parameter.get_value<double>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    value_ = parameter.get_value<std::string>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    value_ = parameter.get_value<bool>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    value_ = parameter.get_value<std::vector<int64_t>>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    value_ = parameter.get_value<std::vector<double>>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    value_ = parameter.get_value<std::vector<std::string>>();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    value_ = parameter.get_value<std::vector<bool>>();
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }
}

Parameter::Parameter(
  rclcpp::Node * node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
  const std::string & name, const uav_cpp::parameters::ParameterType & value,
  const std::string & description)
: uav_cpp::parameters::Parameter(name, value, description)
{
  createRosCallback(node, param_subscriber);
}

void Parameter::createRosCallback(
  rclcpp::Node * node,
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber)
{
  if (cb_handle_) {
    LOG_WARN("Parameter {} already has a ROS callback", name_);
    return;
  }
  ParameterDescriptor descriptor;
  descriptor.description = description_;
  if (std::holds_alternative<int64_t>(value_)) {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = std::get<int64_t>(min_);
    range.to_value = std::get<int64_t>(max_);
    descriptor.set__integer_range({range});
  } else if (std::holds_alternative<double>(value_)) {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = std::get<double>(min_);
    range.to_value = std::get<double>(max_);
    descriptor.set__floating_point_range({range});
  }
  std::visit(
    [this, node, param_subscriber, descriptor](auto && arg) {
      node->declare_parameter(name_, arg, descriptor);
    }, value_);

  cb_handle_ = param_subscriber->add_parameter_callback(
    name_,
    createParameterCallback());
}

void Parameter::createRosCallback(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber)
{
  createRosCallback(node.get(), param_subscriber);
}

std::function<void(const rclcpp::Parameter &)> Parameter::createParameterCallback()
{
  if (std::holds_alternative<int64_t>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             int64_t value = parameter.get_value<int64_t>();
             setValue(value);
           };
  } else if (std::holds_alternative<double>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             double value = parameter.get_value<double>();
             setValue(value);
           };
  } else if (std::holds_alternative<std::vector<int64_t>>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             std::vector<int64_t> value = parameter.get_value<std::vector<int64_t>>();
             setValue(value);
           };
  } else if (std::holds_alternative<std::vector<double>>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             std::vector<double> value = parameter.get_value<std::vector<double>>();
             setValue(value);
           };
  } else if (std::holds_alternative<std::vector<bool>>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             std::vector<bool> value = parameter.get_value<std::vector<bool>>();
             setValue(value);
           };
  } else if (std::holds_alternative<std::vector<std::string>>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             std::vector<std::string> value = parameter.get_value<std::vector<std::string>>();
             setValue(value);
           };
  } else if (std::holds_alternative<bool>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             bool value = parameter.get_value<bool>();
             setValue(value);
           };
  } else if (std::holds_alternative<std::string>(value_)) {
    return [this](const rclcpp::Parameter & parameter) {
             std::string value = parameter.get_value<std::string>();
             setValue(value);
           };
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }
}

ServerParameter::ServerParameter(
  rclcpp::Node * node, std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber,
  const std::string & name, const uav_cpp::parameters::ParameterType & value,
  const std::string & description)
: Parameter(node, param_subscriber, name, value, description)
{
  createRegisterService(node);
}

void ServerParameter::createRegisterService(rclcpp::Node * node)
{
  if (node_) {
    LOG_WARN("Server parameter {} already has a register service", name_);
    return;
  }
  node_ = node;
  // Replace dots with slashes in the parameter name
  std::string service_name = "param/" + name_;
  std::replace(service_name.begin(), service_name.end(), '.', '/');
  service_name += "/register";
  service_name = node->get_fully_qualified_name() + std::string("/") + service_name;

  // Create a service to register a new client node
  register_service_ = node->create_service<ros2_uav_interfaces::srv::ParameterClientRegister>(
    service_name,
    [this](
      const std::shared_ptr<ParameterClientRegister::Request> request,
      std::shared_ptr<ParameterClientRegister::Response> response)
    {
      handleClientRegistration(request, response);
    });
}

void ServerParameter::createRegisterService(rclcpp::Node::SharedPtr node)
{
  createRegisterService(node.get());
}


void ServerParameter::onParameterChange()
{
  std::vector<std::string> invalid_clients;
  for (const auto & client : client_nodes_) {
    auto request_client = client.second;
    // Send the parameter to the client
    try {
      if (!request_client->wait_for_service(1s)) {
        throw std::runtime_error("Service not available");
      }
      auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      rclcpp::Parameter parameter;
      std::visit(
        [&parameter](auto && arg) {
          parameter = rclcpp::Parameter("param_name", arg);
        }, value_);
      auto parameter_msg = parameter.to_parameter_msg();
      request->parameters.push_back(parameter_msg);
      request_client->async_send_request(request);
      LOG_INFO(
        "Sending parameter {} to client {}",
        name_, client.first);
    } catch (const std::exception & e) {
      LOG_ERROR(
        "Failed to send parameter {} to client {}: {}",
        name_, client.first, e.what());
      LOG_WARN(
        "Unregistering client node {} for parameter {}",
        client.first, name_);
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

void ServerParameter::handleClientRegistration(
  const std::shared_ptr<ParameterClientRegister::Request> request,
  std::shared_ptr<ParameterClientRegister::Response> response)
{
  auto client_node_name = request->client_name;
  if (request->register_client) {
    LOG_INFO("Registering client node {} for parameter {}", client_node_name, name_);
    // Check if the client node is already registered
    if (std::find_if(
        client_nodes_.begin(), client_nodes_.end(),
        [&client_node_name](auto & client) {return client.first == client_node_name;}) !=
      client_nodes_.end())
    {
      LOG_WARN("Client node {} is already registered for parameter {}", client_node_name, name_);
      response->success = true;
      response->message = "Client node is already registered";
      return;
    }
    auto client_set_parameters = node_->create_client<rcl_interfaces::srv::SetParameters>(
      client_node_name + "/set_parameters");
    response->success = true;
    client_nodes_.push_back({client_node_name, client_set_parameters});
  } else {
    LOG_INFO("Unregistering client node {} for parameter {}", client_node_name, name_);
    auto it = std::find_if(
      client_nodes_.begin(), client_nodes_.end(),
      [&client_node_name](auto & client) {return client.first == client_node_name;});
    if (it != client_nodes_.end()) {
      client_nodes_.erase(it);
      response->success = true;
    } else {
      LOG_WARN("Client node {} is not registered for parameter {}", client_node_name, name_);
      response->success = false;
      response->message = "Client node is not registered";
    }
  }
}

}  // namespace ros2_uav::parameters
