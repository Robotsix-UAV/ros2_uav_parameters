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

// START_EXAMPLE parameter_client
#include <ros2_uav_parameters/parameter_client.hpp>


// Create a custom Node inherited from ParameterClient
class MyClient : public ros2_uav::parameters::ParameterClient
{
public:
  MyClient(const std::string & node_name, const std::vector<std::string> & required_parameters)
  : ParameterClient(node_name, required_parameters)
  {
    // Create a ROS timer to display the registered parameter values
    auto timer_callback = [this]() {
        double ground_velocity, vertical_velocity;
        remote_parameters_["limits.ground_velocity"]->getValue(ground_velocity);
        remote_parameters_["limits.vertical_velocity"]->getValue(vertical_velocity);
        UAVCPP_INFO(
          "Ground velocity {} --- Vertical velocity {}", ground_velocity,
          vertical_velocity);
      };
    timer_ = create_wall_timer(std::chrono::seconds(1), timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Define the required parameters
  auto required_parameters =
    std::vector<std::string>{"limits.ground_velocity", "limits.vertical_velocity"};

  // Create the custom node
  auto node = std::make_shared<MyClient>(
    "parameter_client",
    required_parameters);

  // Spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// END_EXAMPLE parameter_client
