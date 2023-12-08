#include "nturt_nms1_hardware/nms1_hardware.hpp"

// std include
#include <exception>
#include <memory>
#include <vector>

// ros2 include
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ros2 msg include
#include "std_msgs/msg/float32.hpp"

// jetson gpio include
#include <JetsonGPIO.h>

namespace nms1_hardware {

hardware_interface::CallbackReturn
Nms1Hardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // joint data init
  joint_data_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
    try {
      if (info_.joints[i].parameters.at("type").find("steer") !=
          std::string::npos) {
        joint_data_[i].type = JointType::STEER;
        nr_steer_++;
      } else if (info_.joints[i].parameters.at("type").find("drive") !=
                 std::string::npos) {
        joint_data_[i].type = JointType::DRIVE;
        nr_drive_++;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid joint type for %s",
                     info_.joints[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } catch (std::out_of_range &) {
      RCLCPP_ERROR(node_->get_logger(), "Joint type not specified for %s",
                   info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // node init
  rclcpp::NodeOptions options;
  try {
    options.arguments(
        {"--ros-args", "__ns:=" + info_.hardware_parameters.at("namespace")});
  } catch (std::out_of_range &) {
  }

  node_ = rclcpp::Node::make_shared("nms1_hardware_node", options);
  target_speed_pub_ =
      node_->create_publisher<std_msgs::msg::Float32>("target_speed", 10);
  target_steer_pub_ =
      node_->create_publisher<std_msgs::msg::Float32>("target_steer", 10);
  speed_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "speed", 10,
      std::bind(&Nms1Hardware::onSpeed, this, std::placeholders::_1));

  // gpio init
  GPIO::setmode(GPIO::BOARD);
  // esp32 rst pin is active high so init with low to reset it
  GPIO::setup(EPS32_RST, GPIO::OUT, GPIO::LOW);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Nms1Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  // enable esp32
  GPIO::output(EPS32_RST, GPIO::HIGH);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Nms1Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Nms1Hardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Nms1Hardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  GPIO::cleanup(EPS32_RST);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Nms1Hardware::on_error(const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type Nms1Hardware::prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Nms1Hardware::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
Nms1Hardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.push_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_data_[i].position));

    state_interfaces.push_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_data_[i].velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Nms1Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.push_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_data_[i].position_command));

    command_interfaces.push_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_data_[i].velocity_command));
  }
  return command_interfaces;
}

hardware_interface::return_type
Nms1Hardware::read(const rclcpp::Time & /*time*/,
                   const rclcpp::Duration & /*period*/) {
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  for (auto &joint : joint_data_) {
    switch (joint.type) {
    case JointType::STEER:
      joint.position = target_steer_;
      break;

    case JointType::DRIVE:
      joint.velocity = speed_;
      break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
Nms1Hardware::write(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) {
  target_steer_ = 0;
  float target_speed_ = 0;

  for (auto &joint : joint_data_) {
    switch (joint.type) {
    case JointType::STEER:
      target_steer_ += joint.position_command;
      break;

    case JointType::DRIVE:
      target_speed_ += joint.velocity_command;
      break;
    }
  }

  target_steer_ /= nr_steer_;
  target_speed_ /= nr_drive_;

  std_msgs::msg::Float32 msg;
  msg.data = target_steer_;
  target_steer_pub_->publish(msg);

  msg.data = target_speed_;
  target_speed_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

void Nms1Hardware::onSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
  speed_ = msg->data;
}

} // namespace nms1_hardware

// export definition for pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nms1_hardware::Nms1Hardware,
                       hardware_interface::SystemInterface)
