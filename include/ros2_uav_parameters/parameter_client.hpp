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
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "auto_ros_parameters/parameters.hpp"

namespace ros2_uav::parameters
{
using std::chrono_literals::operator""s;

/**
 * @class ParameterClient
 * @brief A class for managing client-side ROS2 parameters.
 *
 * This class handles the registration and synchronization of parameters
 * with a remote parameter server.
 */
class ParameterClient : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Parameter Client object
   *
   * @param node_name The name of the client node.
   * @param required_parameters A list of required parameters to be synchronized.
   * @param server_name The name of the parameter server node (default is ""). An empty string
   * indicates the default parameter server. (i.e., node_namespace/parameter_server)
   */
  ParameterClient(
    const std::string & node_name, const std::vector<std::string> & required_parameters,
    const std::string & server_name = "");

  /**
   * @brief Destroy the Parameter Client object
   */
  ~ParameterClient();

  /**
   * @brief Register or parameters with the parameter server.
   */
  void registerParameters();

private:
  std::jthread register_thread_;  ///< Thread for registering parameters.
  std::string server_name_;  ///< Name of the parameter server.
  std::vector<std::shared_ptr<ros2_uav::parameters::Parameter>> remote_parameters_;
  ///< List of remote parameters.
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  ///< Parameter event handler subscriber.
};

}  // namespace ros2_uav::parameters
