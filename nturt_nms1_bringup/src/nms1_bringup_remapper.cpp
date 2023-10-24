#include "nturt_nms1_bringup/nms1_bringup_remapper.hpp"

// stl include
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

Nms1BringupRemapper::Nms1BringupRemapper(rclcpp::NodeOptions options)
    : Node("nms1_bringup_remapper", options),
      tf_remapper_(*this, "ackermann_steering_controller/tf_odometry", "/tf"),
      teleop_remapper_(*this, "/cmd_vel",
                       "ackermann_steering_controller/reference_unstamped") {
  this->declare_parameter("my_parameter", "world");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Nms1BringupRemapper)
