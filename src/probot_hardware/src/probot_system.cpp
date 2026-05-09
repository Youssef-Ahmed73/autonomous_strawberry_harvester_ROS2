#include "probot_hardware/probot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace probot_hardware
{

// --- SAFETY MAPPING FUNCTION ---
// Dynamically scales and clamps URDF radians to Physical Servo degrees
int16_t mapToServo(double value, double in_min, double in_max, double out_min, double out_max) {
    // 1. Clamp to prevent hardware damage
    double min_bound = std::min(in_min, in_max);
    double max_bound = std::max(in_min, in_max);
    if (value < min_bound) value = min_bound;
    if (value > max_bound) value = max_bound;

    // 2. Linear map interpolation
    return static_cast<int16_t>(out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min));
}

hardware_interface::CallbackReturn ProbotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_states_.assign(info_.joints.size(), 0.0);

  node_ = std::make_shared<rclcpp::Node>("probot_hw_interface_node");
  
  // =======================================================
  // QOS CONFIGURATION: Best Effort, Keep Last (Depth 1)
  // Ensures only the freshest command is sent over the network
  // =======================================================
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  publisher_arm_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("probot/arm_commands", qos_profile);
  publisher_ee_  = node_->create_publisher<std_msgs::msg::Int16MultiArray>("probot/ee_commands", qos_profile);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ProbotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ProbotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ProbotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProbotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProbotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProbotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto arm_msg = std_msgs::msg::Int16MultiArray();  
  auto ee_msg = std_msgs::msg::Int16MultiArray();  
  
  for (uint i = 0; i < hw_commands_.size(); i++) {
    
    // 1. ARM JOINTS (Indices 0 through 5)
    if (i < 6) {
      double degrees = hw_commands_[i] * (180.0 / M_PI);
      arm_msg.data.push_back(static_cast<int16_t>(degrees));
    }
    
    // 2. END EFFECTOR JOINTS (Indices 6 through 8)
    else {
      // Joint 6: Active Scissors
      if (i == 6) { 
        ee_msg.data.push_back(mapToServo(hw_commands_[i], SCISSORS_URDF_MIN, SCISSORS_URDF_MAX, SCISSORS_PHYS_MIN, SCISSORS_PHYS_MAX));
      }
      // Joint 7: Door 
      else if (i == 7) {
        ee_msg.data.push_back(mapToServo(hw_commands_[i], DOOR_URDF_MIN, DOOR_URDF_MAX, DOOR_PHYS_MIN, DOOR_PHYS_MAX));
      }
      // Joint 8: Active Iris
      else if (i == 8) {
        ee_msg.data.push_back(mapToServo(hw_commands_[i], IRIS_URDF_MIN, IRIS_URDF_MAX, IRIS_PHYS_MIN, IRIS_PHYS_MAX));
      }
    }
  }

  publisher_arm_->publish(arm_msg);
  publisher_ee_->publish(ee_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace probot_hardware

PLUGINLIB_EXPORT_CLASS(
  probot_hardware::ProbotSystemHardware, hardware_interface::SystemInterface)