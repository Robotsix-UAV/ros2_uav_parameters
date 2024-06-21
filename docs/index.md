# ROS2 UAV Parameters Documentation

## Overview

Welcome to the ROS2 UAV Parameters documentation. This repository provides a parameter server node specifically designed for use with other ROS2 UAV packages. The parameter server node is a standalone node that manages UAV parameters within the ROS2 ecosystem. It is designed to handle parameters that are commonly used in the ROS2 UAV nodes. It allows users to find all standard parameters in one place and provides utilities to facilitate interactions with the parameter server.

This documentation provides guidance to users. Developers can find additional information at the [Developer Documentation](rosdoc2/ros2_uav_parameters) page.

## ROS2 Version

This package is designed to work with ROS2 Humble Hawksbill. It has not been tested with other versions of ROS2.

## Features

### Parameter Server Node

A node designed to manage UAV parameters within the ROS2 ecosystem. This node reads parameters from YAML files in the config directory of this package and creates them. They become available to other nodes in the system. Guidance on configuring the parameter server node can be found in the [Configuration](configuration.md) section of this documentation.

You can run the parameter server node using the following commands:

```sh
ros2 run ros2_uav_parameters parameter_server
```

or

```sh
ros2 launch ros2_uav_parameters parameter_server_launch.py config_directory:=<your_config_folder_path>
```

for a custom configuration folder.

### ParameterClientNode

The [ParameterClientNode](parameter_client.md) is a class inherited from ROS2 Node that subscribes to a list of parameters and synchronizes them with the parameter server. It also provides a convenient interface with the uav_cpp::parameters library.
