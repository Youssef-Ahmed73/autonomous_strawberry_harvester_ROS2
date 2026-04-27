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
#include "std_msgs/msg/int8_multi_array.hpp" // CHANGED TO INT8

namespace probot_hardware
{

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
  // CHANGED TO INT8
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_; 
};

}  // namespace probot_hardware

#endif  // PROBOT_HARDWARE__PROBOT_SYSTEM_HPP_