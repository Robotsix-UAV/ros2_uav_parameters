#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_uav_interfaces/srv/parameter_client_register.hpp>
#include <uav_cpp/parameters/parameter.hpp>
#include "ros2_uav_parameters/parameters/parameter.hpp"

using namespace ros2_uav::parameters;

enum class ParamValueType { INT, DOUBLE, STRING, BOOL, INT_ARRAY, DOUBLE_ARRAY, STRING_ARRAY,
  BOOL_ARRAY };

struct TestParam
{
  std::string name;
  ParamValueType type;
  uav_cpp::parameters::ParameterType initial_value;
  uav_cpp::parameters::ParameterType new_value;
  std::string log_message;
};

class ParameterTest : public ::testing::TestWithParam<TestParam>
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("parameter_test_node");
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
};

TEST_P(ParameterTest, ParameterTestWithLogging) {
  TestParam param_info = GetParam();

  Parameter param(node_.get(), param_subscriber_, param_info.name, param_info.initial_value,
    "A parameter");

  // Check initial parameter value
  rclcpp::Parameter rcl_param = node_->get_parameter(param_info.name);
  switch (param_info.type) {
    case ParamValueType::INT:
      EXPECT_EQ(rcl_param.get_value<int64_t>(), std::get<int64_t>(param_info.initial_value));
      break;
    case ParamValueType::DOUBLE:
      EXPECT_EQ(rcl_param.get_value<double>(), std::get<double>(param_info.initial_value));
      break;
    case ParamValueType::STRING:
      EXPECT_EQ(
        rcl_param.get_value<std::string>(),
        std::get<std::string>(param_info.initial_value));
      break;
    case ParamValueType::BOOL:
      EXPECT_EQ(rcl_param.get_value<bool>(), std::get<bool>(param_info.initial_value));
      break;
    case ParamValueType::INT_ARRAY:
      EXPECT_EQ(
        rcl_param.get_value<std::vector<int64_t>>(),
        std::get<std::vector<int64_t>>(param_info.initial_value));
      break;
    case ParamValueType::DOUBLE_ARRAY:
      EXPECT_EQ(
        rcl_param.get_value<std::vector<double>>(),
        std::get<std::vector<double>>(param_info.initial_value));
      break;
    case ParamValueType::STRING_ARRAY:
      EXPECT_EQ(
        rcl_param.get_value<std::vector<std::string>>(),
        std::get<std::vector<std::string>>(param_info.initial_value));
      break;
    case ParamValueType::BOOL_ARRAY:
      EXPECT_EQ(
        rcl_param.get_value<std::vector<bool>>(),
        std::get<std::vector<bool>>(param_info.initial_value));
      break;
  }
  rclcpp::spin_some(node_);

  // Initialize the output capture
  testing::internal::CaptureStdout();

  // Change parameter value
  std::visit(
    [&](auto && arg) {
      node_->set_parameter(rclcpp::Parameter(param_info.name, arg));
    }, param_info.new_value);

  rclcpp::spin_some(node_);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    output.find(param_info.log_message) !=
    std::string::npos) << "Expected log message" << param_info.log_message;
}

INSTANTIATE_TEST_SUITE_P(
  ParameterTests,
  ParameterTest,
  ::testing::Values(
    TestParam{"test_integer_param", ParamValueType::INT, int64_t{42}, int64_t{84},
      "Parameter test_integer_param changed to 84"},
    TestParam{"test_double_param", ParamValueType::DOUBLE, double{3.14}, double{2.71},
      "Parameter test_double_param changed to 2.71"},
    TestParam{"test_string_param", ParamValueType::STRING, std::string{"hello"},
      std::string{"world"}, "Parameter test_string_param changed to world"},
    TestParam{"test_bool_param", ParamValueType::BOOL, bool{true}, bool{false},
      "Parameter test_bool_param changed to false"},
    TestParam{"test_int_array_param", ParamValueType::INT_ARRAY, std::vector<int64_t>{1, 2, 3, 4},
      std::vector<int64_t>{5, 6, 7, 8}, "Parameter test_int_array_param changed to [5, 6, 7, 8]"},
    TestParam{"test_double_array_param", ParamValueType::DOUBLE_ARRAY,
      std::vector<double>{1.1, 2.2, 3.3, 4.4}, std::vector<double>{5.5, 6.6, 7.7, 8.8},
      "Parameter test_double_array_param changed to [5.5, 6.6, 7.7, 8.8]"},
    TestParam{"test_string_array_param", ParamValueType::STRING_ARRAY,
      std::vector<std::string>{"one", "two", "three"},
      std::vector<std::string>{"four", "five", "six"},
      "Parameter test_string_array_param changed to [four, five, six]"},
    TestParam{"test_bool_array_param", ParamValueType::BOOL_ARRAY,
      std::vector<bool>{true, false, true}, std::vector<bool>{false, true, false},
      "Parameter test_bool_array_param changed to [false, true, false]"}
  )
);

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
