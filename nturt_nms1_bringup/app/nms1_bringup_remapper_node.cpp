// stl include
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// nturt include
#include "nturt_nms1_bringup/nms1_bringup_remapper.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto nms1_bringup_remapper_node =
      std::make_shared<Nms1BringupRemapper>(options);

  executor.add_node(nms1_bringup_remapper_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
