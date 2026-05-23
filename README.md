# WELCOME TO ASHR GRADUATION PROJECT 🍓

## About the project
I will write this later XD

## Software architecture
The ASHR project is a modular ROS 2 Humble-based system for autonomous strawberry harvesting, integrating robot motion, perception, and simulation. The architecture is organized into several core packages:

- **probot_description**: Defines the robot’s physical model, kinematics, and sensor frames in URDF/XACRO, serving as the source of truth for all geometry and joint definitions.
- **moveit_config**: Provides the motion planning and controller configuration for the robot, including MoveIt planning groups, controller YAMLs, and kinematics/limits.
- **probot_bringup**: The orchestration layer that launches the robot in simulation or real hardware, wiring together robot description, MoveIt, controllers, and optional Gazebo simulation. It manages conditional startup for both real and simulated environments.
- **probot_hardware**: Implements the hardware interface plugin for ROS 2 control, translating joint commands into actuator messages for the real robot.
- **ashr_simulation**: Supplies Gazebo world files, mesh assets, and bridge configuration for simulation, enabling seamless integration between ROS 2 and the Gazebo simulator.
- **vision**: Handles perception, including camera capture, object detection (using ONNX models), and 3D target localization by synchronizing RGB, depth, and detection streams.

The system is designed for flexibility: you can launch the robot in simulation (with Gazebo and full perception stack) or on real hardware. Communication between components is handled via ROS 2 topics and services, with clear conventions for frames and topic names. Synchronization between perception and planning is achieved using message filters and approximate time policies, ensuring robust operation even with sensor delays.

Key architectural goals:
- Decouple perception, planning, and actuation for easy testing and extension
- Support both simulation and real hardware with minimal launch changes
- Maintain clear frame and topic conventions for reliable integration
- Enable AI-driven perception and planning by exposing all sensor and state data in standard ROS 2 formats

This modular design allows future contributors to extend any layer (e.g., swap in a new vision model, add a new controller, or change the robot geometry) with minimal impact on the rest of the stack.

## Pre-installing configuration
Before you download the repo make sure you:

### 1-Have ros2 Humble installed
You can download it by following this link: 

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 2-Have moveit2 package installed and setted up
You can download it by following this link:

https://youtu.be/c6Bxbq8UdaI?si=s2agZWoYEXX3tex0

## First time installing configurations
Do this only in the first time using the repo
```bash
git clone https://github.com/Youssef-Ahmed73/autonomous_strawberry_harvester_ROS2 ashr_ws
cd ashr_ws
sudo rosdep init  #In case you never did it before, if this gives an error just delete this line
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Starting the robot

You can launch either the simulation or the real robot. In both cases, make sure to source the workspace first.

```bash
cd ashr_ws
source install/setup.bash
```

### Launching gazebo 

Use the following command to start the robot in Gazebo along with all required simulation components:
```bash
ros2 launch probot_bringup gazebo_bringup.launch.py
```

### Launching real hardware
Use the following command to start the robot on real hardware:
```bash
ros2 launch probot_bringup hardware_demo.launch.py
```
Note: At the moment, this launch file starts only the robot hardware interface and does not include the camera nodes.