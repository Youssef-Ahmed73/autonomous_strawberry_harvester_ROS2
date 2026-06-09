# teleoperation

Real-time Xbox 360 joystick teleoperation for the ASHR Probot arm using **MoveIt Servo**.

---

## Package overview

| File | Purpose |
|------|---------|
| `src/joystick_servo_node.cpp` | Subscribes to `/joy`, converts axes/buttons into `TwistStamped` or `JointJog` messages, toggles hardware controllers, and publishes to the servo node |
| `config/servo_config.yaml` | All MoveIt Servo parameters tuned for the Probot arm |
| `launch/joystick_servo.launch.py` | Brings up `joy_node`, `servo_node_main`, and the joystick bridge together |

---

## Control scheme (Xbox 360)

### Cartesian mode (default)

| Input | EEF movement |
|-------|-------------|
| Left stick X | Linear X |
| Left stick Y | Linear Y |
| Right stick Y | Linear Z (up/down) |
| Right stick X | Roll (rotate around X) |
| LT / RT | Pitch (LT = positive, RT = negative) |
| LB / RB | Yaw (rotate around Z) |

### Joint-jog mode

| Input | Joint |
|-------|-------|
| Left stick X | joint_1 |
| Left stick Y | joint_2 |
| Right stick X | joint_3 |
| Right stick Y | joint_4 |
| LT / RT | joint_5 (LT = +, RT = −) |
| LB / RB | joint_6 (LB = +, RB = −) |

### Mode / feature buttons

| Button | Action |
|--------|--------|
| **Back** | Toggle **Cartesian ↔ Joint** mode |
| **Y** | Toggle command frame **base_link ↔ link_6** (Cartesian only) |
| **X** | Toggle Scissor Position |
| **B** | Toggle Door Position |
| **A** | Toggle Iris Position |
| **Start** | Toggle `servo_controller` ↔ `arm_controller` and call `/servo_node/start_servo` service |
| **Left Stick Press** | Deadman switch (optional — see `DEADMAN_ENABLED` in source) |

> **Tip:** In Cartesian mode, switching the command frame to `link_6` makes the
> robot move relative to its own end-effector (useful for fine positioning).
> Switching back to `base_link` makes it move in world-aligned directions.

---

## Dependencies

All dependencies are declared in `package.xml`. Key ones:

- `moveit_servo` (part of `moveit2`)
- `joy` (ROS 2 joystick driver)
- `control_msgs`, `geometry_msgs`, `std_srvs`, `trajectory_msgs`, `controller_manager_msgs`

---

## Building

```bash
cd ashr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select teleoperation
source install/setup.bash