# ashr_autonomy (under construction 🚧)

## Overview
The `ashr_autonomy` package serves as the central "Brain" of the Autonomous Strawberry Harvesting Robot (ASHR) project. It orchestrates the entire harvesting cycle by bridging high-level decision-making with low-level robotic control. 

This package implements a robust, timer-driven Finite State Machine (FSM) written in Python, deeply integrated with the **MoveIt 2 C++ API** via `moveit_py`. It is specifically designed to utilize the **Pilz Industrial Motion Planner** to ensure deterministic, safe, and precise end-effector movements (combining fast Point-to-Point arm swings with strict Cartesian straight-line plunging).

---

## Software Architecture

### The Finite State Machine (FSM) Paradigm
Unlike traditional event-driven architectures which can deadlock if network packets drop, this autonomy stack utilizes a **Timer-Driven Polling** FSM operating at 10Hz. 
* **Continuous Awareness:** The FSM "ticks" every 0.1 seconds, evaluating the current state, checking asynchronous ROS 2 network futures (like target acquisition and ripeness verification), and monitoring hardware execution times.
* **Non-Blocking:** By spinning in a `MultiThreadedExecutor` alongside Mutually Exclusive Callback Groups, the node can block on physical robot trajectories without freezing the ROS 2 network subscriptions.

### Motion Planning: Pilz Industrial Planner
The system strictly enforces industrial planning constraints over standard geometric planning (OMPL).
* **PTP (Point-to-Point):** Used for rapid, curved-path joint-space traversal (e.g., moving from the home position to the strawberry's `pre_grasp_distance`).
* **LIN (Linear):** Used for precision Cartesian straight-line plunging (e.g., engulfing the fruit without swinging the scissors sideways into the bush).

---

## Node Configuration

### 1. `autonomy_fsm_node`
The primary executable containing the state machine and MoveIt 2 integration.
* **Executor:** `MultiThreadedExecutor`
* **API:** `moveit.planning.MoveItPy`
* **Action Clients:** FollowJointTrajectory for `arm_controller` and `ee_controller`.
* **Service Clients:** `/get_harvest_target` (Vision), `/esp1/verify_ripeness`, `/esp2/verify_ripeness` (Edge AI).

### 2. `target_server_node`
A lightweight intermediary state-holder. It subscribes to the asynchronous vision pipeline (`/target_pose`) and serves the most recently detected `PoseStamped` upon request from the FSM via a synchronous/asynchronous service call.

---

## State Machine Breakdown

The FSM transitions through the following states to complete a harvest cycle:

1. **`IDLE`**: The resting state. Monitors basket capacity. Non-blockingly polls the `target_server` for new strawberry coordinates. Transitions to `APPROACH` when a valid `PoseStamped` is acquired.
2. **`APPROACH`**: Extracts the target coordinates, applies the `pre_grasp_distance` offset, configures the MoveIt `PlanRequestParameters` dynamically for the `PTP` algorithm, and executes the trajectory.
3. **`ENGULF`**: Commands the arm to move precisely forward along the Z-axis using the `LIN` algorithm to slide the open end-effector over the fruit.
4. **`INSPECT`**: Halts motion and triggers asynchronous service calls to the ESP32 cameras. Waits for a ripeness score. If ripe, transitions to `HARVEST`. If unripe, transitions to `UNDO_ENGULF`.
5. **`HARVEST`**: Actuates the custom end-effector. Closes the Iris mechanism to secure the stem, closes the Door, and fires the Scissors to cut the stem. Wait timers ensure mechanical actuation completes before moving.
6. **`UNDO_ENGULF`**: The failure-recovery state. If the fruit is unripe or a movement fails, safely backs the arm out along a strict Cartesian line.
7. **`RETREAT`**: After a successful cut, pulls the arm back to clear the bush canopy.
8. **`DROP_OFF`**: Navigates to the basket coordinates, opens the end-effector to release the fruit, increments the harvest counter, and returns to `IDLE`.

---

## Configuration Files

To bypass launch file clutter and parameter injection bugs, configurations are separated into dedicated YAML files installed via `setup.py`:

* **`autonomy_params.yaml`**: Contains FSM logic parameters (`harvests_before_dropoff`, `pre_grasp_distance`, `engulf_plunge_depth`).
* **`moveit_pilz_config.yaml`**: Injects MoveItCpp configurations directly into the Python node, overriding the builder. It defines the `planning_scene_monitor`, loads the `pilz_industrial_motion_planner` pipeline, configures the kinematics adapters (`FixStartStateBounds`, etc.), and provides default scaling factors.

---

## Progress Report & Current Roadmap

### Accomplished Milestones
* [x] Python FSM architecture designed and integrated with `MultiThreadedExecutor`.
* [x] Hardcoded C++ dictionaries removed from `autonomy.launch.py` and modularized into pristine YAML.
* [x] `MoveItPy` API successfully bridged to the underlying `moveit_cpp` engine.
* [x] Pilz planner correctly initialized with `PTP` dynamic assignment resolving the "Empty String" exception.
* [x] Acceleration limits successfully configured to satisfy industrial planning kinematics constraints.
* [x] The `IDLE` state correctly receives a target from the vision pipeline and transitions to `APPROACH`.

### Current Blockers (The TF2 / IK Issue)
The FSM is currently aborting the `APPROACH` state and returning to `IDLE` due to a coordinate frame disconnect. 

**Error Output:**
`Unable to transform from frame 'probot_anno/robot_base/kinect_rgbd' to frame 'world'.`
`Failed to compute inverse kinematics for link: grasp_target_link of goal pose`

**Diagnosis:**
The vision pipeline is identifying the strawberry relative to the camera lens (`kinect_rgbd`). MoveIt is attempting to plan the arm movement relative to the base of the robot (`world`). Because ROS 2's `tf2` tree does not have a static transform connecting the camera to the robot, MoveIt's KDL Inverse Kinematics solver cannot map the mathematical path.

### Next Action Items
To resolve the IK failure and unblock physical movement in Gazebo:

1. **Verify URDF Connectivity:** Check `probot_anno.urdf.xacro`. Ensure there is a `<joint>` (typically `type="fixed"`) connecting the `base_link` (or `world`) to the `probot_anno/robot_base/kinect_rgbd` link.
2. **Verify Target Publisher:** If the URDF is correct, verify that `target_locator_node` is assigning the exact, correct `header.frame_id` to the `PoseStamped` message it sends to the target server. 
3. **TF Tree Debugging:** Run `ros2 run tf2_tools view_frames` while the simulation is active to visually inspect the gap between the camera frame and the arm frames.