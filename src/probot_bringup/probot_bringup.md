# probot_bringup

## Purpose

`probot_bringup` is the orchestration package for launching the Probot system. It assembles robot description, MoveIt, controllers, and optional simulation into a single runtime configuration.

## System architecture

- `probot_bringup.launch.py` is the central launch entrypoint.
- It constructs the robot description and planning configuration via `MoveItConfigsBuilder` from `moveit_config`.
- It starts:
  - `robot_state_publisher`
  - `move_group`
  - `rviz2`
  - `ros2_control_node` when not in Gazebo
  - `ros_gz_sim`, `ros_gz_bridge`, and entity spawn when Gazebo is enabled
- It spawns controllers through `controller_manager`.

## Communication and integration graph

- `robot_state_publisher` publishes TFs from the robot description.
- `move_group` orchestrates motion planning and consumes the same description and controller configs.
- `rviz2` visualizes the planning scene using MoveIt parameters and the robot description.
- `ros2_control_node` provides the hardware interface when `use_gazebo` is false.
- In Gazebo mode, `ros_gz_sim` and `ros_gz_bridge` provide the simulated joint state / actuator interface.
- The launch uses `joint_state_broadcaster`, `arm_controller`, and `ee_controller` to bridge joint commands from MoveIt to the hardware stack.

## Conditional runtime paths

- `use_gazebo=true`:
  - include `ros_gz_sim` launch
  - spawn the robot entity from `probot_description`
  - start `ros_gz_bridge` with `ashr_gazebo/config/bridge.yaml`
  - skip launching `ros2_control_node` explicitly because Gazebo loads its own controller manager
- `use_gazebo=false`:
  - launch `ros2_control_node` with controller YAML from `moveit_config`
  - still run `robot_state_publisher`, `move_group`, and `rviz2`
- `use_sim_time=true` is propagated to all major nodes.

## Synchronization strategy

- Synchronization is handled by ROS 2 node startup order and launch-time parameter propagation.
- The package does not implement explicit topic synchronization; it relies on `move_group` and controller lifecycle management to start after the robot description is available.
- `SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', ...)` is a hidden dependency that must be injected before Gazebo starts.

## Frame and description conventions

- The launch pipeline wires the `probot_anno` model into the planning stack.
- The core TF tree originates from `world` → `robot_base` → `base_footprint` → `base_link`.
- `rviz2` is launched with the MoveIt scene and expects the robot description frames to match the URDF.

## Hidden coupling points

- `MoveItConfigsBuilder` uses the same XACRO file path and parameter names as `moveit_config`.
- `use_mock_hardware`, `use_real_hardware`, `use_gazebo`, and `use_isaac` are passed into the robot description builder and are therefore coupled to XACRO macro parameters.
- The controller spawner names in this launch file must match the names defined in `config/ros2_controllers.yaml`.
- `probot_bringup.launch.py` assumes `ashr_gazebo` provides valid mesh and world assets when `use_gazebo=true`.
- `ros_gz_bridge` depends on `bridge.yaml` mapping topics produced by Gazebo to the expected ROS namespaces.

## Behavioral constraints

- Maintain the conditional split between Gazebo and non-Gazebo hardware paths; collapsing the two without updating all assumptions will break launch behavior.
- Do not rename controller names used by the spawner or you will break the controller manager startup.
- Preserve the environment variable assembly for `IGN_GAZEBO_RESOURCE_PATH`.

## Performance assumptions

- Startup order matters: the robot description must be available before `move_group` and `rviz2` start.
- The system assumes the Gazebo and ROS 2 clocks can be synchronized when `use_sim_time=true`.
- `component_container_mt` is used in vision launch integration for multithreaded components, but this package itself does not directly manage those components.

## Useful details for AI context

- This package is the high-level bootstrap layer of the robot.
- It does not implement controllers or perceptions itself; it delegates to `moveit_config`, `probot_description`, `probot_hardware`, `ashr_gazebo`, and `vision`.
- It is the place to adjust whole-system launch behavior, not individual algorithm behavior.