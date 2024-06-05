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

namespace uav_ros2::utils
{
/**
 * @brief Converts a double to a string and trims trailing zeros.
 *
 * This function takes a double value, converts it to a string,
 * and removes any trailing zeros from the fractional part.
 *
 * @param value The double value to be converted to string.
 * @return A string representation of the double value without trailing zeros.
 */
std::string doubleToStringTrimZero(double value)
{
  std::string str = std::to_string(value);
  str.erase(str.find_last_not_of('0') + 1, std::string::npos);
  return str;
}
}  // namespace uav_ros2::utils
