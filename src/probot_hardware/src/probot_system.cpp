#include "probot_hardware/probot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace probot_hardware
{

hardware_interface::CallbackReturn ProbotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(6, 0.0);
  hw_states_.assign(6, 0.0);

  node_ = std::make_shared<rclcpp::Node>("probot_hw_interface_node");
  // CHANGED TO INT8
  publisher_ = node_->create_publisher<std_msgs::msg::Int8MultiArray>("esp32_arm_commands", 10);

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
  // CHANGED TO INT8
  auto msg = std_msgs::msg::Int8MultiArray();  
  
  for (uint i = 0; i < hw_commands_.size(); i++) {
    // 1. Convert Radians to Degrees
    double degrees = hw_commands_[i] * (180.0 / M_PI);
    // 2. Divide by 4 to compress into Int8 limits
    int8_t compressed_command = static_cast<int8_t>(degrees / 4.0);
    
    msg.data.push_back(compressed_command);
  }

  publisher_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace probot_hardware

PLUGINLIB_EXPORT_CLASS(
  probot_hardware::ProbotSystemHardware, hardware_interface::SystemInterface)