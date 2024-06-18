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
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
import rclpy.service


@pytest.mark.launch_test
def generate_test_description():
    package_name = 'ros2_uav_parameters'
    node_executable = os.path.join(
        get_package_prefix(package_name), 'share', package_name, 'examples/parameter_client'
    )

    node_server = launch_ros.actions.Node(
        package=package_name,
        executable='parameter_server',
        namespace='uav',
        output='screen'
    )

    node = launch_ros.actions.Node(
        package=package_name,
        executable=node_executable,
        namespace='uav',
        output='screen'
    )

    return launch.LaunchDescription([
        node, node_server,
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
        assert wait_for_node(node, 'parameter_client')

    def test_parameter_client(self, proc_output):
        node = rclpy.create_node('test_node')
        assert wait_for_node(node, 'parameter_client')
        client = node.create_client(GetParameters, '/uav/parameter_client/get_parameters')

        ready = client.wait_for_service(timeout_sec=10.0)
        self.assertTrue(ready, 'Service is not available.')

        # Wait for parameters to be available
        time.sleep(1.5)

        request = GetParameters.Request()
        request.names = ['limits.ground_velocity']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        self.assertEqual(response.values[0].type, 3)

        # Set another value in the parameter server and check that the client gets the new value
        parameter_to_set = Parameter()
        parameter_to_set.name = 'limits.ground_velocity'
        parameter_to_set.value.type = 3
        parameter_to_set.value.double_value = 10.0
        # Use the set_parameters service on the server node to set the parameter
        set_param_client = node.create_client(SetParameters, '/uav/parameter_server/set_parameters')

        request_set = rclpy.parameter_service.SetParameters.Request()
        request_set.parameters = [parameter_to_set]

        future = set_param_client.call_async(request_set)

        rclpy.spin_until_future_complete(node, future)

        time.sleep(0.1)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        self.assertEqual(response.values[0].type, 3)
        self.assertEqual(response.values[0].double_value, 10.0)

        node.destroy_node()


def wait_for_node(dummy_node, node_name, timeout=10.0):
    start = time.time()
    flag = False
    print('Waiting for node...')
    while time.time() - start < timeout and not flag:
        flag = node_name in dummy_node.get_node_names()
        time.sleep(0.1)
    return flag
