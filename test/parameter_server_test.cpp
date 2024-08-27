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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_uav_interfaces/srv/parameter_client_register.hpp>
#include <uav_cpp/parameters/parameter.hpp>
#include "ros2_uav_parameters/parameters/parameter.hpp"

using ros2_uav::parameters::Parameter;
using ros2_uav::parameters::ServerParameter;

class ServerParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("server_parameter_test_node");
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
};

TEST_F(ServerParameterTest, ClientRegistrationTest)
{
  ServerParameter server_param(
    node_.get(), param_subscriber_, "server_param", int64_t{100},
    "A server parameter");

  auto client_node = std::make_shared<rclcpp::Node>("client_node");
  auto client = client_node->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
    "server_parameter_test_node/param/server_param/register");
  auto param_subscriber_client = std::make_shared<rclcpp::ParameterEventHandler>(client_node);
  Parameter client_param(client_node.get(), param_subscriber_client, "server_param", int64_t{0});

  // Register client
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client_node->get_fully_qualified_name();
  request->register_client = true;

  auto future = client->async_send_request(request);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(node_);
  executor.spin_until_future_complete(future);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "");

  // Register client again to check for log message
  future = client->async_send_request(request);
  testing::internal::CaptureStdout();
  executor.spin_until_future_complete(future);
  response = future.get();
  EXPECT_TRUE(response->success);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(output.find("already registered") != std::string::npos);

  // Change the parameter value and check for log message
  node_->set_parameter(rclcpp::Parameter("server_param", int64_t{200}));

  rclcpp::Parameter rcl_param;
  for (int i = 0; i < 200; i++) {
    rcl_param = client_node->get_parameter("server_param");
    executor.spin_some();
    if (rcl_param.get_value<int64_t>() == 200) {
      SUCCEED();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "Parameter value did not update on client side";
}

TEST_F(ServerParameterTest, ClientUnregistrationTest)
{
  ServerParameter server_param(
    node_.get(), param_subscriber_, "server_param", int64_t{100},
    "A server parameter");

  auto client_node = std::make_shared<rclcpp::Node>("client_node");
  auto client = client_node->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
    "server_parameter_test_node/param/server_param/register");

  // Register client
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client_node->get_fully_qualified_name();
  request->register_client = true;

  auto future = client->async_send_request(request);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(node_);
  executor.spin_until_future_complete(future);
  auto response = future.get();
  EXPECT_TRUE(response->success);

  // Unregister client
  request->register_client = false;
  future = client->async_send_request(request);
  executor.spin_until_future_complete(future);
  response = future.get();
  EXPECT_TRUE(response->success);

  // Unregister client again
  future = client->async_send_request(request);
  executor.spin_until_future_complete(future);
  response = future.get();
  EXPECT_FALSE(response->success);
}

TEST_F(ServerParameterTest, DestroyClient)
{
  ServerParameter server_param(
    node_.get(), param_subscriber_, "server_param", int64_t{100},
    "A server parameter");

  auto client_node = std::make_shared<rclcpp::Node>("client_node");
  auto client = client_node->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(
    "server_parameter_test_node/param/server_param/register");

  // Register client
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client_node->get_fully_qualified_name();
  request->register_client = true;

  // Spin client in another thread with stop token
  std::jthread client_thread([&client_node](std::stop_token st) {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(client_node);
      while (!st.stop_requested()) {
        executor.spin_some();
      }
    });

  auto future = client->async_send_request(request);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin_until_future_complete(future);
  auto response = future.get();
  EXPECT_TRUE(response->success);

  // Stop client thread and destroy client node
  client_thread.request_stop();
  client_thread.join();
  client_node.reset();

  rclcpp::Parameter rcl_param;
  for (int i = 0; i < 200; i++) {
    node_->set_parameter(rclcpp::Parameter("server_param", int64_t{200}));
    testing::internal::CaptureStdout();
    executor.spin_some();
    std::string output = testing::internal::GetCapturedStdout();
    if (output.find("Unregistering client node") != std::string::npos) {
      SUCCEED();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "Expected automatic client unregistration did not occur";
}

TEST_F(ServerParameterTest, CreateSecondRegisterService)
{
  ServerParameter server_param(
    node_, param_subscriber_, "server_param", int64_t{100},
    "A server parameter");
  testing::internal::CaptureStdout();
  server_param.createRegisterService(node_);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    output.find("already has a register service") !=
    std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
