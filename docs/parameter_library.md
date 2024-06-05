## Parameters Library Documentation

### Examples

Compiling examples using the parameter library can be found in the [examples](https://github.com/Robotsix-UAV/ros2_uav_parameters/tree/main/examples) directory of the repository.


### YamlParameterParser

#### Usage

The `YamlParameterParser` class is used to parse YAML files containing parameter definitions. It reads the parameters from the specified YAML files and stores them for use in the ROS 2 parameter server as a vector of tuples containing the parameter name, value, and descriptor.

#### Example Usage

<!-- INSERT_EXAMPLE: yaml_parser_usage -->

### uav_ros2::Parameter

The `uav_ros2::Parameter` class is used to manage ROS 2 parameters. It automatically subscribes to parameter changes and provides a callback method to handle parameter changes. The class also provides a method to log parameter changes. It is intended to be used with the `rclcpp::Node` class.

#### Modifying the callback

The `uav_ros2::Parameter` class provides a default callback method that logs parameter changes. You can modify this callback method by subclassing the `uav_ros2::Parameter` class and overriding the `onParameterChanged` method.

<!-- INSERT_EXAMPLE: parameters_callback -->

#### Adding a new parameter

Here is a minimal example of how to add a new parameter using the `uav_ros2::Parameter` class or a subclass:

<!-- INSERT_EXAMPLE: parameters_usage -->

#### Advanced usage
An example of a more advanced usage of the `uav_ros2::Parameter` class can be found in the [parameter_server.cpp](https://github.com/Robotsix-UAV/ros2_uav_parameters/blob/main/src/pparameter_server.cpp) file.