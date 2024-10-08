# ROS2 UAV Parameters Documentation

## Overview

Welcome to the ROS2 UAV Parameters documentation. This repository provides a parameter server node specifically designed for use with other ROS2 UAV packages. The parameter server node is a standalone node that manages UAV parameters within the ROS2 ecosystem. It is designed to handle parameters that are commonly used in the ROS2 UAV nodes. It allows users to find all standard parameters in one place.

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

### ParameterClientNode

The `ParameterClientNode` is a class inherited from ROS2 Node that subscribes to a list of parameters and synchronizes them with the parameter server. It stores the parameters under a std::map remote_parameters_ that can be accessed by the node.

<!-- INSERT_EXAMPLE: parameter_client -->