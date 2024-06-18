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

// START_EXAMPLE parameter_client
#include <auto_ros_parameters/parameter_client.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto required_parameters =
    std::vector<std::string>{"limits.ground_velocity", "limits.vertical_velocity"};

  auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "parameter_client",
    required_parameters);

  rclcpp::spin(parameter_client_node);
  rclcpp::shutdown();
  return 0;
}
// END_EXAMPLE parameter_client
