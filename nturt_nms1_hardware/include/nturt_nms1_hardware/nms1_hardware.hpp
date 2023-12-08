// std include
#include <memory>
#include <vector>

// ros2 include
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ros2 msg include
#include "std_msgs/msg/float32.hpp"

#define EPS32_RST 12

/// @brief Namespace required by ros2_control hardware components.
namespace nms1_hardware {

enum class JointType {
  STEER = 0,
  DRIVE,
};

struct JointData {
  JointType type;

  double position = 0;

  double velocity = 0;

  double position_command = 0;

  double velocity_command = 0;
};

class Nms1Hardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Nms1Hardware)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void onSpeed(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_steer_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

  std::vector<JointData> joint_data_;

  float speed_ = 0;

  float target_steer_ = 0;

  float nr_steer_ = 0;

  float nr_drive_ = 0;
};

} // namespace nms1_hardware
