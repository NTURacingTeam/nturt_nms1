#include "nturt_nms1_hardware/nms1_hardware.hpp"

namespace nms1_hardware {

hardware_interface::CallbackReturn Nms1Hardware::on_init(
    const hardware_interface::HardwareInfo& info) {}

hardware_interface::CallbackReturn Nms1Hardware::on_configure(
    const rclcpp_lifecycle::State& previous_state) {}

std::vector<hardware_interface::StateInterface>
Nms1Hardware::export_state_interfaces() {}

std::vector<hardware_interface::CommandInterface>
Nms1Hardware::export_command_interfaces() {}

hardware_interface::CallbackReturn Nms1Hardware::on_activate(
    const rclcpp_lifecycle::State& previous_state) {}

hardware_interface::CallbackReturn Nms1Hardware::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {}

hardware_interface::return_type Nms1Hardware::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {}

hardware_interface::return_type Nms1Hardware::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {}

}  // namespace nms1_hardware

// export definition for pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nms1_hardware::Nms1Hardware,
                       hardware_interface::SystemInterface)
