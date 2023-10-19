// stl include
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <tf2_msgs/msg/tf_message.hpp>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for remapping ackermann steering controller.
 */
class AckermannSteeringControllerRepammer : public rclcpp::Node {
 public:
  /// @brief Constructor of AckermannSteeringControllerRepammer.
  AckermannSteeringControllerRepammer(rclcpp::NodeOptions options)
      : Node("ackermann_steering_controller_repammer", options),
        tf_pub_(this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10)),
        tf_sub_(this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/nms1/ackermann_steering_controller/tf_odometry", 10,
            std::bind(&AckermannSteeringControllerRepammer::onTF, this,
                      std::placeholders::_1))) {}

 private:
  void onTF(const std::shared_ptr<tf2_msgs::msg::TFMessage> _msg) {
    tf2_msgs::msg::TFMessage msg = *_msg;
    tf_pub_->publish(msg);
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

/* entry function ------------------------------------------------------------*/
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto ackermann_steering_controller_repammer_node =
      std::make_shared<AckermannSteeringControllerRepammer>(options);

  executor.add_node(ackermann_steering_controller_repammer_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
