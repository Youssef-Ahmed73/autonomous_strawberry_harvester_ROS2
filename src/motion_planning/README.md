# motion_planning

Real-time Xbox 360 joystick teleoperation for the ASHR Probot arm using **MoveIt Servo**.

---

## Package overview

| File | Purpose |
|------|---------|
| `src/joystick_servo_node.cpp` | Subscribes to `/joy`, converts axes/buttons into `TwistStamped` or `JointJog` messages, publishes to the servo node |
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
| D-pad X | Yaw (rotate around Z) |

### Joint-jog mode

| Input | Joint |
|-------|-------|
| Left stick X | joint_1 |
| Left stick Y | joint_2 |
| Right stick X | joint_3 |
| Right stick Y | joint_4 |
| LT / RT | joint_5 (LT = +, RT = −) |
| D-pad X | joint_6 |

### Mode / feature buttons

| Button | Action |
|--------|--------|
| **A** | Toggle **Cartesian ↔ Joint** mode |
| **Y** | Toggle command frame **base_link ↔ link_6** (Cartesian only) |
| **Start** | Call `/servo_node/start_servo` service |
| **Back** | Deadman switch (optional — see `DEADMAN_ENABLED` in source) |

> **Tip:** In Cartesian mode, switching the command frame to `link_6` makes the
> robot move relative to its own end-effector (useful for fine positioning).
> Switching back to `base_link` makes it move in world-aligned directions.

---

## Dependencies

All dependencies are declared in `package.xml`. Key ones:

- `moveit_servo` (part of `moveit2`)
- `joy` (ROS 2 joystick driver)
- `control_msgs`, `geometry_msgs`, `std_srvs`

---

## Building

```bash
cd ashr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select motion_planning
source install/setup.bash
```

---

## Running

### Step 1 – Start simulation (as usual)
```bash
ros2 launch probot_bringup gazebo_bringup.launch.py
```

### Step 2 – Start the joystick teleoperation
```bash
ros2 launch motion_planning joystick_servo.launch.py
```

If your joystick is not on `/dev/input/js0`:
```bash
ros2 launch motion_planning joystick_servo.launch.py joy_dev:=/dev/input/js1
```

### Step 3 – Activate Servo

Press **Start** on the controller, **or** run:
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```

The robot will now follow joystick commands.

---

## Tuning

All speed scaling lives in `config/servo_config.yaml` under `scale:`:

```yaml
scale:
  linear:     0.4   # max EEF linear speed  [m/s]
  rotational: 0.4   # max EEF angular speed [rad/s]
  joint:      0.4   # max joint speed       [rad/s]
```

Lower these values for slower, more precise motion. The dead-zone threshold
(default `0.10`) can be changed in `joystick_servo_node.cpp` in the `deadzone()`
helper function.

### Enabling the deadman switch

For real-hardware safety, open `src/joystick_servo_node.cpp` and change:

```cpp
static constexpr bool DEADMAN_ENABLED = false;
```
to
```cpp
static constexpr bool DEADMAN_ENABLED = true;
```

Then rebuild. With this on, **Back** must be held at all times or the node
will not publish any commands.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot doesn't move | Servo not started | Press Start or call the service |
| `Joy message has fewer axes…` warning | Wrong controller / driver | Check `ros2 topic echo /joy` axes count |
| `planning_frame not found` | `base_link` not in TF | Make sure `probot_bringup` is fully up first |
| Servo stops after ~0.1 s | `incoming_command_timeout` reached | Check that `joy_node` is publishing at ≥10 Hz |
| Collision stop | Robot near collision/singularity | Move away manually or reset pose in simulation |
