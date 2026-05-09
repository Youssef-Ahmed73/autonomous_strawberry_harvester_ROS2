#ifndef PROBOT_HARDWARE__PROBOT_SYSTEM_HPP_
#define PROBOT_HARDWARE__PROBOT_SYSTEM_HPP_

#include <vector>
#include <memory>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int16_multi_array.hpp" 

namespace probot_hardware
{

// =======================================================
// DYNAMIC SERVO MAPPING MACROS
// Configure specific physical limits for each joint
// =======================================================

// 1. Active Scissors
#define SCISSORS_URDF_MIN 0.0
#define SCISSORS_URDF_MAX 0.5
#define SCISSORS_PHYS_MIN 0.0    // Physical Servo degree for URDF Min
#define SCISSORS_PHYS_MAX 180.0  // Physical Servo degree for URDF Max

// 2. Door
#define DOOR_URDF_MIN -3.14
#define DOOR_URDF_MAX 0.0
#define DOOR_PHYS_MIN 180        // Physical Servo degree for URDF Min (-3.14)
#define DOOR_PHYS_MAX 0      // Physical Servo degree for URDF Max (0.0)

// 3. Active Iris
#define IRIS_URDF_MIN 0.0
#define IRIS_URDF_MAX 1.57
#define IRIS_PHYS_MIN 0.0        // Physical Servo degree for URDF Min
#define IRIS_PHYS_MAX 180.0      // Physical Servo degree for URDF Max

class ProbotSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::shared_ptr<rclcpp::Node> node_;
  
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_arm_; 
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_ee_; 
};

}  // namespace probot_hardware

#endif  // PROBOT_HARDWARE__PROBOT_SYSTEM_HPP_