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
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include "auto_ros_parameters/parameters.hpp"

using ros2_uav::parameters::Parameter;
using ros2_uav::parameters::ServerParameter;

class ParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
    rclcpp::spin_some(node_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
};

class TestParameter : public Parameter
{
public:
  using Parameter::Parameter;

  void setOnChangeCallback(std::function<void(const rclcpp::Parameter &)> callback)
  {
    on_change_callback_ = callback;
  }

protected:
  void onParameterChange(const rclcpp::Parameter & parameter) override
  {
    if (on_change_callback_) {
      on_change_callback_(parameter);
    }
  }

private:
  std::function<void(const rclcpp::Parameter &)> on_change_callback_;
};

class TestServerParameter : public ServerParameter
{
public:
  using ServerParameter::ServerParameter;

  void setOnChangeCallback(std::function<void(const rclcpp::Parameter &)> callback)
  {
    on_change_callback_ = callback;
  }

protected:
  void onParameterChange(const rclcpp::Parameter & parameter) override
  {
    if (on_change_callback_) {
      on_change_callback_(parameter);
    }
  }

private:
  std::function<void(const rclcpp::Parameter &)> on_change_callback_;
};

using ParameterTestTuple = std::tuple<std::string, rclcpp::Parameter, rclcpp::ParameterType>;

class ParameterParameterizedTest : public ParameterTest,
  public ::testing::WithParamInterface<ParameterTestTuple>
{
};

TEST_P(ParameterParameterizedTest, DeclareAndChangeParameter)
{
  auto [param_name, initial_param, param_type] = GetParam();

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Test parameter";

  int callback_triggered = 0;

  TestParameter test_param(
    node_, param_handler_, param_name, initial_param.get_value<rclcpp::ParameterValue>(),
    descriptor);

  test_param.setOnChangeCallback(
    [&callback_triggered, param_name, param_type, initial_param](
      const rclcpp::Parameter & parameter)
    {
      ++callback_triggered;
      EXPECT_EQ(parameter.get_name(), param_name);
      EXPECT_EQ(parameter.get_type(), param_type);
      EXPECT_EQ(parameter, initial_param);
    });

  rclcpp::spin_some(node_);

  // Update parameter with the same value to trigger the callback
  node_->set_parameter(initial_param);
  rclcpp::spin_some(node_);

  EXPECT_EQ(callback_triggered, 2);
}

TEST_P(ParameterParameterizedTest, ServerParameterRegisterAndChange)
{
  auto [param_name, initial_param, param_type] = GetParam();

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Test server parameter";

  int callback_triggered = 0;

  TestServerParameter server_param(
    node_, param_handler_, param_name, initial_param.get_value<rclcpp::ParameterValue>(),
    descriptor);

  server_param.setOnChangeCallback(
    [&callback_triggered, param_name, param_type, initial_param](
      const rclcpp::Parameter & parameter)
    {
      ++callback_triggered;
      EXPECT_EQ(parameter.get_name(), param_name);
      EXPECT_EQ(parameter.get_type(), param_type);
      EXPECT_EQ(parameter, initial_param);
    });

  rclcpp::spin_some(node_);

  // Register a client
  auto client = std::make_shared<rclcpp::Node>("client_node");
  auto param_handler_client = std::make_shared<rclcpp::ParameterEventHandler>(client);

  // Create the parameter also in the client
  TestParameter test_param(
    client, param_handler_client, param_name, initial_param.get_value<rclcpp::ParameterValue>(),
    descriptor);

  // Set the client callback
  int callback_triggered_client = 0;
  test_param.setOnChangeCallback(
    [&callback_triggered_client, param_name, param_type, initial_param](
      const rclcpp::Parameter & parameter)
    {
      ++callback_triggered_client;
      EXPECT_EQ(parameter.get_name(), param_name);
      EXPECT_EQ(parameter.get_type(), param_type);
      EXPECT_EQ(parameter, initial_param);
    });

  std::string service_name = "/test_node/param/" + param_name;
  std::replace(service_name.begin(), service_name.end(), '.', '/');
  service_name += "/register";
  auto client_register_service =
    client->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(service_name);
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client->get_fully_qualified_name();
  request->register_client = true;

  ASSERT_TRUE(client_register_service->wait_for_service(std::chrono::seconds(1)));
  auto future_result = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response = future_result.get();
  EXPECT_TRUE(response->success);

  // Update parameter with the same value to trigger the callbacks
  node_->set_parameter(initial_param);

  while (callback_triggered_client < 2) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(client);
  }

  EXPECT_EQ(callback_triggered, 2);
  EXPECT_EQ(callback_triggered_client, 2);

  // Unregister the client
  request->register_client = false;
  auto future_result_unregister = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response_unregister = future_result_unregister.get();
  EXPECT_TRUE(response_unregister->success);
}

TEST_F(ParameterTest, ServerParameterRegisterTwice)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Test server parameter";

  TestServerParameter server_param(
    node_, param_handler_, "test_param", 10, descriptor);

  rclcpp::spin_some(node_);

  // Register a client
  auto client = std::make_shared<rclcpp::Node>("client_node");

  std::string service_name = "/test_node/param/test_param";
  std::replace(service_name.begin(), service_name.end(), '.', '/');
  service_name += "/register";
  auto client_register_service =
    client->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(service_name);
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client->get_fully_qualified_name();
  request->register_client = true;

  ASSERT_TRUE(client_register_service->wait_for_service(std::chrono::seconds(1)));
  auto future_result = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response = future_result.get();
  EXPECT_TRUE(response->success);

  // Register the client again
  auto future_result_twice = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response_twice = future_result_twice.get();
  EXPECT_TRUE(response_twice->success);
  EXPECT_EQ(response_twice->message, "Client node is already registered");
}

TEST_F(ParameterTest, ServerParameterUnregister)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Test server parameter";

  TestServerParameter server_param(
    node_, param_handler_, "test_param", 10, descriptor);

  rclcpp::spin_some(node_);

  auto client = std::make_shared<rclcpp::Node>("client_node");

  std::string service_name = "/test_node/param/test_param";
  std::replace(service_name.begin(), service_name.end(), '.', '/');
  service_name += "/register";
  auto client_register_service =
    client->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(service_name);
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client->get_fully_qualified_name();
  request->register_client = true;

  ASSERT_TRUE(client_register_service->wait_for_service(std::chrono::seconds(1)));

  // Unregister the client
  request->register_client = false;
  auto future_result_unregister = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response_unregister_twice = future_result_unregister.get();
  EXPECT_FALSE(response_unregister_twice->success);
  EXPECT_EQ(response_unregister_twice->message, "Client node is not registered");
}

TEST_F(ParameterTest, ServerParameterAutomaticUnregister)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Test server parameter";

  TestServerParameter server_param(
    node_, param_handler_, "test_param", 10, descriptor);

  rclcpp::spin_some(node_);

  auto client = std::make_shared<rclcpp::Node>("client_node");

  std::string service_name = "/test_node/param/test_param";
  std::replace(service_name.begin(), service_name.end(), '.', '/');
  service_name += "/register";
  auto client_register_service =
    client->create_client<ros2_uav_interfaces::srv::ParameterClientRegister>(service_name);
  auto request = std::make_shared<ros2_uav_interfaces::srv::ParameterClientRegister::Request>();
  request->client_name = client->get_fully_qualified_name();
  request->register_client = true;

  ASSERT_TRUE(client_register_service->wait_for_service(std::chrono::seconds(1)));
  auto future_result = client_register_service->async_send_request(request);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(client);

  auto response = future_result.get();
  EXPECT_TRUE(response->success);

  // Unregister the client
  client.reset();
  rclcpp::spin_some(node_);

  // Capture the log messages
  testing::internal::CaptureStderr();
  // Update parameter to trigger the automatic unregister
  node_->set_parameter(rclcpp::Parameter("test_param", 20));
  // Spin ros for 2 s to trigger the automatic unregister
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    rclcpp::spin_some(node_);
  }
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_NE(output.find("Unregistering client node"), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  ParameterTests,
  ParameterParameterizedTest,
  ::testing::Values(
    ParameterTestTuple(
      "test_param_bool", rclcpp::Parameter("test_param_bool", true),
      rclcpp::ParameterType::PARAMETER_BOOL),
    ParameterTestTuple(
      "test_param_int", rclcpp::Parameter("test_param_int", 10),
      rclcpp::ParameterType::PARAMETER_INTEGER),
    ParameterTestTuple(
      "test_param_double", rclcpp::Parameter("test_param_double", 10.0),
      rclcpp::ParameterType::PARAMETER_DOUBLE),
    ParameterTestTuple(
      "test_param_string",
      rclcpp::Parameter("test_param_string", std::string("test")),
      rclcpp::ParameterType::PARAMETER_STRING),
    ParameterTestTuple(
      "test_param_bool_array",
      rclcpp::Parameter("test_param_bool_array", std::vector<bool>{true, false}),
      rclcpp::ParameterType::PARAMETER_BOOL_ARRAY),
    ParameterTestTuple(
      "test_param_int_array", rclcpp::Parameter(
        "test_param_int_array",
        std::vector<int64_t>{1, 2, 3}), rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY),
    ParameterTestTuple(
      "test_param_double_array",
      rclcpp::Parameter("test_param_double_array", std::vector<double>{1.1, 2.2, 3.3}),
      rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY),
    ParameterTestTuple(
      "test_param_string_array",
      rclcpp::Parameter("test_param_string_array", std::vector<std::string>{"one", "two", "three"}),
      rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
));

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
