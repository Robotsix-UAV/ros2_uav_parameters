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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "auto_ros_parameters/parameter_client.hpp"

using rcl_interfaces::msg::ParameterDescriptor;

class ParameterServer : public rclcpp::Node
{
public:
  ParameterServer()
  : Node("parameter_server")
  {
    parameter_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param1 = std::make_shared<ros2_uav::parameters::ServerParameter>(
      this, parameter_handler_, "param1", 1, ParameterDescriptor());
    param2 = std::make_shared<ros2_uav::parameters::ServerParameter>(
      this, parameter_handler_, "param2", 2, ParameterDescriptor());
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_handler_;
  std::shared_ptr<ros2_uav::parameters::ServerParameter> param1;
  std::shared_ptr<ros2_uav::parameters::ServerParameter> param2;
};

class BadServerParameter : public rclcpp::Node
{
public:
  BadServerParameter()
  : Node("bad_parameter_server")
  {
    declare_parameter("param1", 1);
    declare_parameter("param2", 2);
  }
};

class TestParameterClient : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Start parameter server
    parameter_server_node_ = std::make_shared<ParameterServer>();
    parameter_server_thread_ = std::jthread(
      [this](std::stop_token st) {
        while (!st.stop_requested() && rclcpp::ok()) {
          rclcpp::spin_some(parameter_server_node_);
        }
      });

    // Required parameters for ParameterClient
    required_parameters_ = {"param1", "param2"};
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (parameter_server_thread_.joinable()) {
      parameter_server_thread_.join();
    }
  }

  std::shared_ptr<rclcpp::Node> parameter_server_node_;
  std::jthread parameter_server_thread_;
  std::vector<std::string> required_parameters_;
};

TEST_F(TestParameterClient, RegisterWithParameterServer)
{
  // Create ParameterClient node
  auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "parameter_client",
    required_parameters_);

  // Wait for the ParameterClient node to register the parameters
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parameter_client_node);

  // Allow some time for the ParameterClient to register
  executor.spin_until_future_complete(
    std::promise<bool>().get_future(),
    std::chrono::milliseconds(100));

  // Check if the parameters were registered with the appropriate values
  auto param1 = parameter_client_node->get_parameter("param1");
  EXPECT_EQ(param1.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(param1.as_int(), 1);

  auto param2 = parameter_client_node->get_parameter("param2");
  EXPECT_EQ(param2.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(param2.as_int(), 2);

  // Set the parameter in parameter server and check if the ParameterClient receives it
  parameter_server_node_->set_parameter(rclcpp::Parameter("param1", 10));
  int max_iterations = 20;
  while (parameter_client_node->get_parameter("param1").as_int() != 10 && max_iterations-- > 0) {
    executor.spin_some();
  }
  param1 = parameter_client_node->get_parameter("param1");
  EXPECT_EQ(param1.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(param1.as_int(), 10);

  // Set the parameter in parameter server and check if the ParameterClient receives it
  parameter_server_node_->set_parameter(rclcpp::Parameter("param2", 20));
  max_iterations = 20;
  while (parameter_client_node->get_parameter("param2").as_int() != 20 && max_iterations-- > 0) {
    executor.spin_some();
  }
  param2 = parameter_client_node->get_parameter("param2");
  EXPECT_EQ(param2.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(param2.as_int(), 20);
}

TEST_F(TestParameterClient, RegisterWithParameterServerMissingParameter)
{
  // Create ParameterClient node with missing parameter
  required_parameters_.push_back("param3");
  EXPECT_THROW(
    auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
      "parameter_client",
      required_parameters_),
    rclcpp::exceptions::InvalidParametersException);
}

TEST_F(TestParameterClient, NoServer)
{
  try {
    auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
      "parameter_client",
      required_parameters_,
      "non_existent_server");
    FAIL();
  } catch (const std::exception & e) {
    std::string expected_error = "Parameter server non_existent_server not available";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

class TestParameterClientBadServer : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Required parameters for ParameterClient
    required_parameters_ = {"param1", "param2"};
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (bad_server_thread_.joinable()) {
      bad_server_thread_.join();
    }
  }

  std::shared_ptr<BadServerParameter> bad_server_node_;
  std::jthread bad_server_thread_;
  std::vector<std::string> required_parameters_;
};

using ParameterClientBadServerDeathTest = TestParameterClientBadServer;

TEST_F(ParameterClientBadServerDeathTest, RegisterWithBadServer)
{
  // Create ParameterClient node
  ASSERT_DEATH(
    rclcpp::init(0, nullptr);

    // Start parameter server
    bad_server_node_ = std::make_shared<BadServerParameter>();
    bad_server_thread_ = std::jthread(
      [this](std::stop_token st) {
        while (!st.stop_requested() && rclcpp::ok()) {
          rclcpp::spin_some(bad_server_node_);
        }
      });
    auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
      "parameter_client",
      required_parameters_,
      "bad_parameter_server"),
    "Failed to connect to parameter server service for parameter param1");
}

TEST_F(ParameterClientBadServerDeathTest, RegisterWithWrongService)
{
  ASSERT_DEATH(
    rclcpp::init(0, nullptr);
    // Start parameter server
    bad_server_node_ = std::make_shared<BadServerParameter>();
    auto register_service_ =
    bad_server_node_->create_service<ros2_uav_interfaces::srv::ParameterClientRegister>(
      "bad_parameter_server/param/param1/register",
      [](const std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Request>,
      std::shared_ptr<ros2_uav_interfaces::srv::ParameterClientRegister::Response> response)
      {
        response->success = false;
        return;
      });
    bad_server_thread_ = std::jthread(
      [this](std::stop_token st) {
        while (!st.stop_requested() && rclcpp::ok()) {
          rclcpp::spin_some(bad_server_node_);
        }
      });

    auto parameter_client_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
      "parameter_client",
      required_parameters_,
      "bad_parameter_server");
    rclcpp::spin(parameter_client_node),
    "Failed to register client in parameter server");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
