#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_demo_hardware/diffbot_system.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"
#include "std_msgs/msg/string.hpp"

namespace ros2_control_demo_hardware {

class HardwareCommandPub
    : public rclcpp::Node  // the node definition for the publisher to talk to
                           // micro-ROS agent
{
 public:
  HardwareCommandPub();
  void publishData();

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class DiffBotSystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

  std::shared_ptr<HardwareCommandPub>
      hw_cmd_pub_;  // make the publisher node a member

 private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;
};

HardwareCommandPub::HardwareCommandPub() : Node("hardware_command_publisher") {
  publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);
}

void HardwareCommandPub::publishData() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! ";
  publisher_->publish(message);
}

CallbackReturn DiffBotSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  hw_cmd_pub_ =
      std::make_shared<HardwareCommandPub>();  // fire up the publisher node

  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code
  hw_start_sec_ =
      stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
      stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code
  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffBotSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffBotSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn DiffBotSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
                "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
                "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read() {
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Reading...");

  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels
  double dt = 0.01;      // Control period
  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[1] + dt * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    // START: This part here is for exemplary purposes - Please do not copy to
    // your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!",
                hw_positions_[i], hw_velocities_[i],
                info_.joints[i].name.c_str());
    // END: This part here is for exemplary purposes - Please do not copy to
    // your production code
  }

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx =
      0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy =
      0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * dt;
  base_y_ += base_dy * dt;
  base_theta_ += base_dtheta * dt;

  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Joints successfully read! (%.5f,%.5f,%.5f)", base_x_, base_y_,
              base_theta_);
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ros2_control_demo_hardware::DiffBotSystemHardware::write() {
  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
                "Got command %.5f for '%s'!", hw_commands_[i],
                info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
              "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  hw_cmd_pub_->publishData();  // publish to topic
  return hardware_interface::return_type::OK;
}
}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_control_demo_hardware::DiffBotSystemHardware,
                       hardware_interface::SystemInterface)
