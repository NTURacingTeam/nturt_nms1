/**
 * @file nms1_bringup_remapper.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef NMS1_BRINGUP_REMAPPER_HPP
#define NMS1_BRINGUP_REMAPPER_HPP

// stl include
#include <functional>
#include <memory>
#include <string>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for remapping a topic to another by first subscribing to it then
 * publishing the same message to the remapped topic.
 */
template <typename MessageT>
class Remapper {
 public:
  /// @brief Default construct for Remapper.
  Remapper() = default;

  /**
   * @brief Constructor for Remapper.
   *
   * @param[in,out] node ROS2 node.
   * @param[in] from_topic Topic to remap from.
   * @param[in] to_topic Topic to remap to.
   * @param[in] buffer_depth Buffer size of the publisher and subscriber.
   */
  Remapper(rclcpp::Node &node, std::string from_topic, std::string to_topic,
           int buffer_depth = 10)
      : pub_(node.create_publisher<MessageT>(to_topic, buffer_depth)),
        sub_(node.create_subscription<MessageT>(
            from_topic, buffer_depth,
            std::bind(&Remapper::onMsg, this, std::placeholders::_1))) {}

  /**
   * @brief Remap a topic to another.
   *
   * @param[in,out] node ROS2 node.
   * @param[in] from_topic Topic to remap from.
   * @param[in] to_topic Topic to remap to.
   * @param[in] buffer_depth Buffer size of the publisher and subscriber.
   */
  void remap(rclcpp::Node &node, std::string from_topic, std::string to_topic,
             int buffer_depth = 10) {
    pub_ = node.create_publisher<MessageT>(to_topic, buffer_depth);
    sub_ = node.create_subscription<MessageT>(
        from_topic, buffer_depth,
        std::bind(&Remapper::onMsg, this, std::placeholders::_1));
  }

 private:
  /// @brief Callback function when receiving message.
  void onMsg(const std::shared_ptr<MessageT> msg) { pub_->publish(*msg); }

  /// @brief Publisher.
  std::shared_ptr<rclcpp::Publisher<MessageT>> pub_;

  /// @brief Subscriber.
  std::shared_ptr<rclcpp::Subscription<MessageT>> sub_;
};

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for remapping nms1 bringup.
 */
class Nms1BringupRemapper : public rclcpp::Node {
 public:
  /// @brief Constructor of Nms1BringupRemapper.
  Nms1BringupRemapper(rclcpp::NodeOptions options);

 private:
  /// @brief Remapper for ackermann_steering_controller transform.
  Remapper<tf2_msgs::msg::TFMessage> tf_remapper_;

  /// @brief Remapper for ackermann_steering_controller command.
  Remapper<geometry_msgs::msg::Twist> teleop_remapper_;
};

#endif // NMS1_BRINGUP_REMAPPER_HPP
