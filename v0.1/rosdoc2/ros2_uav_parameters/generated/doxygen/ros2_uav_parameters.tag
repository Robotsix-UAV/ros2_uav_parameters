<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.1">
  <compound kind="class">
    <name>uav_ros2::Parameter</name>
    <filename>classuav__ros2_1_1Parameter.html</filename>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classuav__ros2_1_1Parameter.html</anchorfile>
      <anchor>ab22c0b52efdb9f860fa368402a39b712</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_suscriber, const std::string &amp;name, const T &amp;value, const ParameterDescriptor &amp;descriptor)</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>onParameterChange</name>
      <anchorfile>classuav__ros2_1_1Parameter.html</anchorfile>
      <anchor>ac4a578af524b8f48488468d9e134d811</anchor>
      <arglist>([[maybe_unused]] const rclcpp::Parameter &amp;parameter)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Node::SharedPtr</type>
      <name>node_</name>
      <anchorfile>classuav__ros2_1_1Parameter.html</anchorfile>
      <anchor>afc8c600c61dd16522ebba2d1bbef9619</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>uav_ros2::YamlParameterParser</name>
    <filename>classuav__ros2_1_1YamlParameterParser.html</filename>
    <member kind="typedef">
      <type>std::variant&lt; int, double, std::string, bool, std::vector&lt; int &gt;, std::vector&lt; double &gt;, std::vector&lt; std::string &gt;, std::vector&lt; bool &gt; &gt;</type>
      <name>ParameterValue</name>
      <anchorfile>classuav__ros2_1_1YamlParameterParser.html</anchorfile>
      <anchor>a67dba94ca8246184ab221181cefac775</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>YamlParameterParser</name>
      <anchorfile>classuav__ros2_1_1YamlParameterParser.html</anchorfile>
      <anchor>a53d3ccbc4570a0457e1b657be0ffffe0</anchor>
      <arglist>(const std::string &amp;file_path)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::tuple&lt; std::string, ParameterValue, rcl_interfaces::msg::ParameterDescriptor &gt; &gt; &amp;</type>
      <name>getParameters</name>
      <anchorfile>classuav__ros2_1_1YamlParameterParser.html</anchorfile>
      <anchor>a6b9f1bb385f9aa69116a62e636f9f0b7</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
</tagfile>
