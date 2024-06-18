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
#include "auto_ros_parameters/utils.hpp"

namespace ros2_uav::utils
{

// Test cases for doubleToStringTrimZero
class DoubleToStringTrimZeroTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(DoubleToStringTrimZeroTest, HandlesNoTrailingZeros)
{
  double value = 123.456;
  std::string expected = "123.456";
  std::string result = doubleToStringTrimZero(value);
  EXPECT_EQ(result, expected);
}

TEST_F(DoubleToStringTrimZeroTest, HandlesTrailingZeros)
{
  double value = 123.456000;
  std::string expected = "123.456";
  std::string result = doubleToStringTrimZero(value);
  EXPECT_EQ(result, expected);
}

TEST_F(DoubleToStringTrimZeroTest, HandlesWholeNumber)
{
  double value = 123.0;
  std::string expected = "123.";
  std::string result = doubleToStringTrimZero(value);
  EXPECT_EQ(result, expected);
}

TEST_F(DoubleToStringTrimZeroTest, HandlesZeroValue)
{
  double value = 0.0;
  std::string expected = "0.";
  std::string result = doubleToStringTrimZero(value);
  EXPECT_EQ(result, expected);
}

TEST_F(DoubleToStringTrimZeroTest, HandlesNegativeValue)
{
  double value = -123.456000;
  std::string expected = "-123.456";
  std::string result = doubleToStringTrimZero(value);
  EXPECT_EQ(result, expected);
}

}  // namespace ros2_uav::utils

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
