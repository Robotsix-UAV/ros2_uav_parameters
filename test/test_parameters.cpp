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

namespace uav_ros2
{

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

}  // namespace uav_ros2

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
