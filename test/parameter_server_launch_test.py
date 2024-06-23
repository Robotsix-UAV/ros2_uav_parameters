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

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    # Retrieve the package share directory
    package_share_directory = get_package_share_directory(
        'ros2_uav_parameters')
    config_directory = os.path.join(package_share_directory, 'config')

    parameter_server_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'ros2_uav_parameters',
             'parameter_server_launch.py', f'config_directory:={config_directory}'],
        output='screen'
    )

    return launch.LaunchDescription([
        parameter_server_launch,
        launch_testing.actions.ReadyToTest()
    ]), {'parameter_server_launch': parameter_server_launch}


def test_parameter_server_launch_running(parameter_server_launch, proc_info):
    proc_info.assertWaitForStartup(process=parameter_server_launch, timeout=10)

    # Initialize rclpy
    rclpy.init()
    node = rclpy.create_node('test_node')

    try:
        # Check if the parameter server node is up
        node_names = [name for name,
                      namespace in node.get_node_names_and_namespaces()]
        assert 'parameter_server_node' in node_names
    finally:
        # Shutdown rclpy
        rclpy.shutdown()

    proc_info.assertWaitForShutdown(
        process=parameter_server_launch, timeout=10)
