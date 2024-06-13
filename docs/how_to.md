# How to

## Get parameters from the parameter server

To get parameters from the parameter server, you can use the `rclcpp::SyncParametersClient` class. This class provides a synchronous interface to the parameter server, allowing you to get parameters from the server in a blocking manner.

```cpp
auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(my_node, "parameter_server");
while (!parameters_client->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
}
auto param_int = parameters_client->get_parameter<int>("param");
```

There is also a non-blocking/asynchronous version `rclcpp::AsyncParametersClient`.
