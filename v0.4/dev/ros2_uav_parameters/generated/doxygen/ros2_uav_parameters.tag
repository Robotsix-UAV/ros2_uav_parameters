<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.11.0" doxygen_gitid="9b424b03c9833626cd435af22a444888fbbb192d">
  <compound kind="class">
    <name>ros2_uav::parameters::Parameter</name>
    <filename>classros2__uav_1_1parameters_1_1Parameter.html</filename>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>afbf4113be77e4edacb50cb7aaaad5ee1</anchor>
      <arglist>(rclcpp::Node *node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>a5a1466f3506ee3f5a210829ce2a70eb6</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>a3bc3fc873d2c233d08469d163bc93d43</anchor>
      <arglist>(const rclcpp::Parameter &amp;parameter)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createRosCallback</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>a7e9621ca0d1aec48b661fc70bae9e42b</anchor>
      <arglist>(rclcpp::Node *node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createRosCallback</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>a4b8c3d8893df9d12ada5b945784690de</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::function&lt; void(const rclcpp::Parameter &amp;)&gt;</type>
      <name>createParameterCallback</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>a9f3bf5bc9c3bc17b10119607f708aa52</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::ParameterCallbackHandle &gt;</type>
      <name>cb_handle_</name>
      <anchorfile>classros2__uav_1_1parameters_1_1Parameter.html</anchorfile>
      <anchor>af1669b679dfff99827ca7f05c02cf72c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::parameters::ParameterClient</name>
    <filename>classros2__uav_1_1parameters_1_1ParameterClient.html</filename>
    <member kind="function">
      <type></type>
      <name>ParameterClient</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ParameterClient.html</anchorfile>
      <anchor>acf5f53b62abf2137704e4d44a091705d</anchor>
      <arglist>(const std::string &amp;node_name, const std::vector&lt; std::string &gt; &amp;required_parameters, const std::string &amp;server_name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ParameterClient</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ParameterClient.html</anchorfile>
      <anchor>aa1149fba7c70ef29a5e6085c12b57d73</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>registerParameters</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ParameterClient.html</anchorfile>
      <anchor>a4da0ec8c5e2b5a956783e915c0d8dade</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ParameterMap</type>
      <name>getParameters</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ParameterClient.html</anchorfile>
      <anchor>a7bd60254e7c479d87c8009936121f069</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ParameterMap</type>
      <name>remote_parameters_</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ParameterClient.html</anchorfile>
      <anchor>a76ddd1631d94ca9826c08ff089db8009</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::parameters::ServerParameter</name>
    <filename>classros2__uav_1_1parameters_1_1ServerParameter.html</filename>
    <base>ros2_uav::parameters::Parameter</base>
    <member kind="function">
      <type></type>
      <name>ServerParameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>a7654148a1ffb79645066bb7caf475429</anchor>
      <arglist>(rclcpp::Node *node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ServerParameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>ad8b6768ebc143d0d922474962a417e54</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createRegisterService</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>af48e7df793c6c26c62660bc212a6d1b2</anchor>
      <arglist>(rclcpp::Node *node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createRegisterService</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>a12c47983a008ecf90c3ed2946ecd0a20</anchor>
      <arglist>(rclcpp::Node::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>afbf4113be77e4edacb50cb7aaaad5ee1</anchor>
      <arglist>(rclcpp::Node *node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>a5a1466f3506ee3f5a210829ce2a70eb6</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, std::shared_ptr&lt; rclcpp::ParameterEventHandler &gt; param_subscriber, const std::string &amp;name, const uav_cpp::parameters::ParameterType &amp;value, const std::string &amp;description=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classros2__uav_1_1parameters_1_1ServerParameter.html</anchorfile>
      <anchor>a3bc3fc873d2c233d08469d163bc93d43</anchor>
      <arglist>(const rclcpp::Parameter &amp;parameter)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::parameters</name>
    <filename>namespaceros2__uav_1_1parameters.html</filename>
    <class kind="class">ros2_uav::parameters::Parameter</class>
    <class kind="class">ros2_uav::parameters::ParameterClient</class>
    <class kind="class">ros2_uav::parameters::ServerParameter</class>
  </compound>
</tagfile>
