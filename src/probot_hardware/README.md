# probot_hardware

## Purpose

`probot_hardware` is the `ros2_control` plugin package for the Probot arm. It converts high-level joint commands from the controller stack into hardware-level servo command messages.

## System architecture

- Exports a shared library implementing `hardware_interface::SystemInterface`.
- It is loaded by `controller_manager` via `pluginlib` using `probot_hardware.xml`.
- It does not create a standalone ROS node for control; it creates an internal `rclcpp::Node` only to publish command topics.

## Communication graph

- Exports position state interfaces for each robot joint.
- Exports position command interfaces for each robot joint.
- Publishes:
  - `probot/arm_commands` (`std_msgs/msg/Int16MultiArray`)
  - `probot/ee_commands` (`std_msgs/msg/Int16MultiArray`)
- Uses `rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()` for command publication, indicating a low-latency, lossy hardware command path.

## Joint mapping conventions

- Joints `0-5` are mapped to arm joint degrees and published to `probot/arm_commands`.
- Joint `6` maps to the scissors actuator.
- Joint `7` maps to the door actuator.
- Joint `8` maps to the iris actuator.
- The end-effector joint mapping is hard-coded in `write()` and depends on URDF joint order and naming.

## Hidden coupling points

- Joint naming/order must match the URDF and controller configuration.
- The `ProbotSystemHardware` plugin is referenced in `probot_description/urdf/probot.ros2_control.xacro`.
- `write()` assumes the command array length and joint indices directly correspond to the robot’s high-level joints.
- `read()` currently mirrors commands back to states without reading actual hardware, so the hardware interface is effectively open-loop for state values.
- The safety mapping constants for scissors, door, and iris must stay in sync with URDF joint limits and physical actuator ranges.

## Behavioral constraints

- If a new joint is added, update `export_state_interfaces()`, `export_command_interfaces()`, and the `write()` mapping logic.
- Maintain the end-effector index mapping for arm vs. EE joints; changing it requires updating both robot model and hardware plugin logic.
- Preserve the best-effort QoS and single-history command semantics if hardware timing is expected to be low-latency.
- Do not assume valid feedback from real hardware: `read()` currently uses the last command as state.

## Performance assumptions

- The plugin is intended to run at controller update rates and expects `ros2_control_node` to call `read()`/`write()` cyclically.
- It assumes servo command publishing is cheap and that the downstream actuator consumer can handle `Int16MultiArray` messages.
- The code prioritizes safe command clamping over complex joint-space transforms.

## Useful details for AI context

- This package is the boundary between abstract joint control and physical actuator command publication.
- It is a plugin rather than a node; its lifecycle is driven by `ros2_control` and `controller_manager`.
- Any changes to joint indexing or actuator mapping ripple through `probot_description`, `moveit_config`, and the bringup launch stack.
