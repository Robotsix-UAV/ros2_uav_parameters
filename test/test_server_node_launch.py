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

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
import yaml


valid_data = True


def create_test_config(invalid=False):
    test_dir = 'test_config'
    os.makedirs(test_dir, exist_ok=True)
    temp_file_path = os.path.join(test_dir, 'test_config.yaml')
    if invalid:
        config = {
            'Parameter_group_1': {
                'Parameter1': [1, 2, 3]
            }
        }
    else:
        config = {
            'Parameter_group_1': {
                'Parameter1': {
                    'default': 5,
                    'min': 0,
                    'max': 10,
                    'type': 'int',
                    'description': 'An integer parameter for testing',
                    'constraints_description': 'Value must be between 0 and 10'
                }
            }
        }
    with open(temp_file_path, 'w') as temp_file:
        yaml.dump(config, temp_file)
    return test_dir


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'dir_valid',
            default_value='OK',
            choices=['OK', 'Empty', 'NotExists', 'Invalid'],
            description='Validity of the configuration folder'
        ),
        launch.actions.OpaqueFunction(
            function=lambda context: set_node_arguments(context)),
        launch_testing.actions.ReadyToTest()
    ])


def set_node_arguments(context):
    global valid_data
    dir_valid = context.launch_configurations['dir_valid']
    if dir_valid == 'OK':
        parameters = [{'config_directory': create_test_config()}]
    elif dir_valid == 'Empty':
        valid_data = False
        parameters = []
    elif dir_valid == 'NotExists':
        valid_data = False
        parameters = [{'config_directory': '/path/that/does/not/exist'}]
    else:
        valid_data = False
        parameters = [{'config_directory': create_test_config(True)}]

    return [
        launch_ros.actions.Node(
            package='ros2_uav_parameters',
            executable='parameter_server',
            name='ros2_uav_parameters',
            parameters=parameters
        )
    ]


class TestFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_start(self, proc_output):
        node = Node('test_node')
        node_up = wait_for_node(node, 'ros2_uav_parameters', 10.0)
        # Node can go down if the arguments are invalid
        if valid_data:
            assert node_up, 'Node should be up'
        node.destroy_node()

    def test_parameter_initialization(self, proc_output):
        global valid_data

        node = Node('test_node')
        client = node.create_client(GetParameters, 'ros2_uav_parameters/get_parameters')

        ready = client.wait_for_service(timeout_sec=10.0)
        if valid_data:
            self.assertTrue(ready, 'Service is not available.')
        else:
            if not ready:
                self.skipTest('Service is not available as expected.')

        request = GetParameters.Request()
        request.names = ['Parameter_group_1.Parameter1']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        if valid_data:
            self.assertNotEqual(len(response.values), 0)
            self.assertEqual(response.values[0].type, 2)
            self.assertEqual(response.values[0].integer_value, 5)
        else:
            self.assertEqual(len(response.values), 0)

        node.destroy_node()


def wait_for_node(dummy_node, node_name, timeout=10.0):
    start = time.time()
    flag = False
    print('Waiting for node...')
    while time.time() - start < timeout and not flag:
        flag = node_name in dummy_node.get_node_names()
        time.sleep(0.1)
    return flag
