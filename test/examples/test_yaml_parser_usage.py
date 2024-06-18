# Copyright 2024 Damien SIX (damien@robotsix.net)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    package_name = 'ros2_uav_parameters'
    yaml_parser_executable = os.path.join(
        get_package_prefix(package_name), 'share', package_name,  'examples/yaml_parser_usage'
    )

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[yaml_parser_executable],
            name='yaml_parser',
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestYamlParserExecution(unittest.TestCase):

    def test_yaml_parser_runs(self, proc_output):
        proc_output.assertWaitFor('Parameter name: group1.parameter1', timeout=5, stream='stdout')
        proc_output.assertWaitFor('Parameter name: group1.parameter2', timeout=5, stream='stdout')
