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
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions
import launch_testing
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    package_name = 'ros2_uav_parameters'
    node_executable = os.path.join(
        get_package_prefix(package_name), 'share', package_name, 'examples/parameters_usage'
    )

    node = launch_ros.actions.Node(
        package=package_name,
        executable=node_executable,
        output='screen'
    )

    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest()
    ]), {'node': node}


class TestNodeParameter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_up_and_running(self, proc_output):
        node = rclpy.create_node('test_node')
        assert wait_for_node(node, 'parameters_usage')

    def test_parameter_exists(self, proc_output):
        node = rclpy.create_node('test_node')
        assert wait_for_node(node, 'parameters_usage')
        client = node.create_client(GetParameters, 'parameters_usage/get_parameters')

        ready = client.wait_for_service(timeout_sec=10.0)
        self.assertTrue(ready, 'Service is not available.')

        request = GetParameters.Request()
        request.names = ['my_custom_parameter']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        self.assertEqual(response.values[0].type, 2)
        self.assertEqual(response.values[0].integer_value, 42)

    def test_callback_output(self, proc_output):
        proc_output.assertWaitFor(
            'The answer to the Ultimate Question of Life, the Universe, and Everything!',
            timeout=10
        )


def wait_for_node(dummy_node, node_name, timeout=10.0):
    start = time.time()
    flag = False
    print('Waiting for node...')
    while time.time() - start < timeout and not flag:
        flag = node_name in dummy_node.get_node_names()
        time.sleep(0.1)
    return flag
