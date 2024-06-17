# Parameters Library Documentation

## Examples

Compiling examples using the parameter library can be found in the `examples` directory of the repository.

## ros2_uav::parameters::YamlParameterParser

### YamlParameterParserUsage

The `YamlParameterParser` class is used to parse YAML files containing parameter definitions. It reads the parameters from the specified YAML files and stores them for use in the ROS 2 parameter server as a vector of tuples containing the parameter name, value, and descriptor.

### Example Usage

<!-- INSERT_EXAMPLE: yaml_parser_usage -->

## ros2_uav::parameters::Parameter

The `Parameter` class is used to manage ROS 2 parameters. It automatically subscribes to parameter changes and provides a callback method to handle parameter changes. The class also provides a method to log parameter changes. It is intended to be used with the `rclcpp::Node` class.

### Modifying the Callback

The `Parameter` class provides a default callback method that logs parameter changes. You can modify this callback method by subclassing the `Parameter` class and overriding the `onParameterChange` method.

<!-- INSERT_EXAMPLE: parameters_callback -->

### Adding a New Parameter

Here is a minimal example of how to add a new parameter using the `Parameter` class or a subclass:

<!-- INSERT_EXAMPLE: parameters_usage -->

## ros2_uav::parameters::ServerParameter

The `ServerParameter` class inherits from the `Parameter` class and provides additional functionality. It creates a ROS 2 service using the convention `/<parameter_server_name>/param/<parameter_name>/<group_name>/register` to register a parameter with the parameter server. When a client node is registered with the parameter server, the parameter server will send the current value of the parameter to the client node each time the parameter is updated.

### ServerParameter Usage

An example of an advanced usage of the `ServerParameter` class can be found in the `parameter_server.cpp` file.

## ros2_uav::parameters::ParameterClient

The `ParameterClient` class manages client-side ros2_uav::parameters. It registers parameters with a remote parameter server and synchronizes their values. The `ParameterClient` class handles the registration and unregistration of parameters, ensuring that the parameters on the client are updated when they change on the server.

<!-- INSERT_EXAMPLE: parameter_client -->
