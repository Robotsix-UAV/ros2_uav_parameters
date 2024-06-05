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
#include <fstream>
#include <vector>
#include <string>
#include <variant>
#include "auto_ros_parameters/yaml_parameter_parser.hpp"

class YamlParameterParserTest : public ::testing::Test
{
protected:
  // Helper method to create a temporary YAML file
  std::string createTempYamlFile(const std::string & content)
  {
    std::string filename = "/tmp/temp_test.yaml";
    std::ofstream out(filename);
    out << content;
    out.close();
    return filename;
  }

  // Helper method to get parameter by name
  template<typename T>
  T getParameterValue(
    const std::vector<std::tuple<std::string,
    uav_ros2::YamlParameterParser::ParameterValue,
    rcl_interfaces::msg::ParameterDescriptor>> & parameters,
    const std::string & name)
  {
    for (const auto & param : parameters) {
      if (std::get<0>(param) == name) {
        return std::get<T>(std::get<1>(param));
      }
    }
    throw std::runtime_error("Parameter not found: " + name);
  }
};

TEST_F(YamlParameterParserTest, ValidYamlFile)
{
  std::string yaml_content =
    R"(
group1:
  param_int:
    default: 10
    type: int
  param_double:
    default: 3.14
    type: double
  param_string:
    default: "test"
    type: string
  param_bool:
    default: true
    type: bool
  param_int_array:
    default: [1, 2, 3]
    type: int
  param_double_array:
    default: [1.1, 2.2, 3.3]
    type: double
  param_string_array:
    default: ["one", "two", "three"]
    type: string
  param_bool_array:
    default: [true, false, true]
    type: bool
)";
  std::string filename = createTempYamlFile(yaml_content);

  uav_ros2::YamlParameterParser parser(filename);
  auto parameters = parser.getParameters();

  ASSERT_EQ(parameters.size(), (size_t)8);

  // Verify parameter values
  EXPECT_EQ(getParameterValue<int>(parameters, "group1.param_int"), 10);
  EXPECT_EQ(getParameterValue<double>(parameters, "group1.param_double"), 3.14);
  EXPECT_EQ(getParameterValue<std::string>(parameters, "group1.param_string"), "test");
  EXPECT_EQ(getParameterValue<bool>(parameters, "group1.param_bool"), true);
  EXPECT_EQ(
    getParameterValue<std::vector<int>>(parameters, "group1.param_int_array"),
    (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(
    getParameterValue<std::vector<double>>(parameters, "group1.param_double_array"),
    (std::vector<double>{1.1, 2.2, 3.3}));
  EXPECT_EQ(
    getParameterValue<std::vector<std::string>>(parameters, "group1.param_string_array"),
    (std::vector<std::string>{"one", "two", "three"}));
  EXPECT_EQ(
    getParameterValue<std::vector<bool>>(parameters, "group1.param_bool_array"),
    (std::vector<bool>{true, false, true}));
}

TEST_F(YamlParameterParserTest, ParameterWithRanges)
{
  std::string yaml_content =
    R"(
group1:
  param_int:
    default: 10
    type: int
    min: 0
    max: 20
  param_double:
    default: 3.14
    type: double
    min: 1.0
    max: 5.0
)";
  std::string filename = createTempYamlFile(yaml_content);

  uav_ros2::YamlParameterParser parser(filename);
  auto parameters = parser.getParameters();

  ASSERT_EQ(parameters.size(), (size_t)2);

  // Verify parameter values
  EXPECT_EQ(getParameterValue<int>(parameters, "group1.param_int"), 10);
  EXPECT_EQ(getParameterValue<double>(parameters, "group1.param_double"), 3.14);

  // Verify ranges
  auto int_descriptor = std::get<2>(parameters[0]);
  auto double_descriptor = std::get<2>(parameters[1]);

  ASSERT_NE(int_descriptor.integer_range.size(), 0);
  ASSERT_NE(double_descriptor.floating_point_range.size(), 0);

  EXPECT_EQ(int_descriptor.integer_range[0].from_value, 0);
  EXPECT_EQ(int_descriptor.integer_range[0].to_value, 20);

  EXPECT_EQ(double_descriptor.floating_point_range[0].from_value, 1.0);
  EXPECT_EQ(double_descriptor.floating_point_range[0].to_value, 5.0);
}

TEST_F(YamlParameterParserTest, ParameterWithConstraints)
{
  std::string yaml_content =
    R"(
group1:
  param_string:
    default: "test"
    type: string
    description: "A test string parameter"
    constraints_description: "Must be non-empty"
)";
  std::string filename = createTempYamlFile(yaml_content);

  uav_ros2::YamlParameterParser parser(filename);
  auto parameters = parser.getParameters();

  ASSERT_EQ(parameters.size(), (size_t)1);

  // Verify parameter values
  EXPECT_EQ(getParameterValue<std::string>(parameters, "group1.param_string"), "test");

  // Verify constraints
  auto string_descriptor = std::get<2>(parameters[0]);

  EXPECT_EQ(string_descriptor.description, "A test string parameter");
  EXPECT_EQ(string_descriptor.additional_constraints, "Must be non-empty");
}

TEST_F(YamlParameterParserTest, MissingRequiredFields)
{
  std::string yaml_content = R"(
group1:
  param1:
    type: int
)";

  try {
    std::string filename = createTempYamlFile(yaml_content);
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error =
      "Parameter group1.param1 is missing required 'default' or 'type' field.";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

TEST_F(YamlParameterParserTest, InvalidYamlStructure)
{
  std::string yaml_content = R"(
group1:
  - param1: 10
)";
  std::string filename = createTempYamlFile(yaml_content);

  try {
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error =
      "Invalid YAML file. Parameter groups must be maps.";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

TEST_F(YamlParameterParserTest, EmptyYamlFile)
{
  std::string yaml_content = "";
  std::string filename = createTempYamlFile(yaml_content);

  try {
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error = "Invalid YAML file. Top level must be a map.";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

TEST_F(YamlParameterParserTest, InvalidParameterType)
{
  std::string yaml_content = R"(
group1:
  param1:
    default: 10
    type: unknown_type
)";
  std::string filename = createTempYamlFile(yaml_content);

  try {
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error =
      "Parameter group1.param1 has an invalid type: unknown_type";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

TEST_F(YamlParameterParserTest, InvalidParameterArrayType)
{
  std::string yaml_content = R"(
group1:
  param1:
    default: [1, 2, 3]
    type: unknown_type
)";
  std::string filename = createTempYamlFile(yaml_content);

  try {
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error =
      "Parameter group1.param1 has an invalid type: unknown_type";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

TEST_F(YamlParameterParserTest, MalformedDefaultValue)
{
  std::string yaml_content = R"(
group1:
  param1:
    default:
      value: 10
    type: int
)";

  try {
    std::string filename = createTempYamlFile(yaml_content);
    uav_ros2::YamlParameterParser parser(filename);
    FAIL() << "Expected exception not thrown";
  } catch (const std::exception & e) {
    std::string expected_error =
      "Invalid YAML file. Parameter default value must be a scalar or a sequence.";
    EXPECT_STREQ(e.what(), expected_error.c_str());
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
