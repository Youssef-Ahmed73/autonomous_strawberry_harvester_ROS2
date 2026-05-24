# moveit_config

## Purpose

`moveit_config` defines the motion planning and controller configuration for the Probot arm. It connects the robot description to MoveIt, controller YAML, and runtime launch logic.

## System architecture

This package is the planning/configuration layer of the robot stack:

- It exposes a robot description XACRO (`probot_anno.urdf.xacro`) that is loaded into MoveIt and `robot_state_publisher`.
- It defines `ros2_control` controller configuration in `config/ros2_controllers.yaml`.
- It provides MoveIt execution parameters and planner settings through `config/moveit_controllers.yaml`, `config/kinematics.yaml`, `config/joint_limits.yaml`.
- It contains launch files that instantiate MoveIt, the planner, RViz, and controller spawners.

## Communication graph

- `robot_state_publisher` is provided by the launch files in this package, publishing TFs from `robot_description`.
- `move_group` consumes the robot description and controller definitions and publishes planning actions, joint trajectories, and state updates.
- `controller_manager` uses the controller configuration defined here to spawn controllers named:
  - `joint_state_broadcaster`
  - `arm_controller`
  - `ee_controller`
- The package is the source of truth for joint limits, kinematics, and planning groups that MoveIt uses.

## Key contents

- `config/probot_anno.urdf.xacro` - robot description source for planning and control
- `config/probot_anno.ros2_control.xacro` - `ros2_control` robot model that includes optional Isaac-specific topic mapping and `use_isaac` gating
- `config/ros2_controllers.yaml` - controller definitions for `ros2_control_node`, including `servo_controller`
- `config/moveit_controllers.yaml` - MoveIt trajectory execution settings
- `config/kinematics.yaml` - inverse kinematics solver parameters
- `config/joint_limits.yaml` - joint limit guardrails used by MoveIt
- `config/moveit.rviz` - MoveIt/RViz visualization startup state
- `launch/` - MoveIt and simulation launch compositions
- `package.xml` - declares dependencies on MoveIt, robot_state_publisher, and `probot_description`

## Synchronization and runtime assumptions

- The package assumes `robot_description` is available before `move_group` starts.
- It assumes the controller YAML names in `ros2_controllers.yaml` match the controller spawners launched by `probot_bringup`.
- Static virtual joint TF launches and `moveit.rviz` files must stay aligned with the robot model.
- The planner assumes the MoveIt robot description uses the same joint names and semantics as the URDF/XACRO from `probot_description`.

## Frame conventions

- The robot description uses `world` → `robot_base` → `base_footprint` → `base_link` as the fixed root chain.
- Joints `joint_1` through `joint_6` represent the arm revolute joints.
- The end-effector / tool frame is exposed through `tool0` and must remain stable for planning and grasp targeting.

## Hidden coupling points

- `probot_bringup.launch.py` expects the MoveIt config to provide the robot description and controller YAMLs.
- XACRO parameter names such as `use_mock_hardware`, `use_real_hardware`, `use_gazebo`, and `use_isaac` are passed through the launch layer and must remain consistent across packages.
- Controller names in `ros2_controllers.yaml` are implicitly coupled to launch-time spawner arguments and `controller_manager` service paths.
- `moveit.rviz` assumes the same planning groups and robot_description sources as the launch pipeline.

## Behavioral constraints

- Do not rename or remove joint names used in the planner and controller YAMLs without updating all dependent launch files and URDF/XACRO mappings.
- Preserve the robot description parameters used by `MoveItConfigsBuilder`.
- Keep the semantic and planning groups stable; changing the MoveIt model structure requires updating both planners and visualization configs.

## Performance assumptions

- Real-time execution is expected on `ros2_control_node` using the supplied controller definitions.
- The planner depends on valid joint limit and kinematics tuning to avoid generating infeasible trajectories.
- The package assumes MoveIt 2 can parse the XACRO and controller configs within the same ROS environment.
- `setup_assistant.launch.py` exists for config editing, but runtime behavior depends on the on-disk YAML structure remaining coherent.
