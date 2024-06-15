#pragma once

#include <rclcpp/rclcpp.hpp>

namespace uav_ros2
{
using std::chrono_literals::operator""s;

class ParameterClient : public rclcpp::Node
{
public:
  ParameterClient(
    const std::string & node_name, std::vector<std::string> & required_parameters,
    const std::string & server_name = "/parameter_server")
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
    remote_parameters_ = parameter_client->get_parameters(required_parameters, 1s);
  }

  std::vector<rclcpp::Parameter> remote_parameters_;
};
} // namespace uav_ros2
