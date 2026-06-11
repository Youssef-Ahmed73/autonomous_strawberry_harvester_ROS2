# WELCOME TO ASHR GRADUATION PROJECT 🍓

## About the project
I will write this later XD

## Software architecture
The ASHR project is a modular ROS 2 Humble-based system for autonomous strawberry harvesting, integrating robot motion, perception, and simulation. The architecture is organized into several core packages:

- **probot_description**: Defines the robot’s physical model, kinematics, and sensor frames in URDF/XACRO, serving as the source of truth for all geometry and joint definitions.
- **moveit_config**: Provides the motion planning and controller configuration for the robot, including MoveIt planning groups, controller YAMLs, and kinematics/limits.
- **teleoperation**: Provides joystick teleoperation and MoveIt Servo integration for the Probot arm, translating Xbox controller inputs into motion commands.
- **probot_bringup**: The orchestration layer that launches the robot in simulation or real hardware, wiring together robot description, MoveIt, controllers, and optional Gazebo simulation. It manages conditional startup for both real and simulated environments.
- **probot_hardware**: Implements the hardware interface plugin for ROS 2 control, translating joint commands into actuator messages for the real robot.
- **ashr_simulation**: Supplies Gazebo world files, Isaac/ROS 2 simulation assets, and bridge configuration for simulation, enabling seamless integration between ROS 2 and the simulator.
- **vision**: Handles perception, including camera capture, object detection (using ONNX models), and 3D target localization by synchronizing RGB, depth, and detection streams.
- **ashr_interfaces**: Defines the custom ROS 2 messages, services, and actions used for inter-package communication across the system (e.g., ripeness verification).

The system is designed for flexibility: you can launch the robot in simulation (with Gazebo and full perception stack) or on real hardware. Communication between components is handled via ROS 2 topics and services, with clear conventions for frames and topic names. Synchronization between perception and planning is achieved using message filters and approximate time policies, ensuring robust operation even with sensor delays.

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

# Starting the Software stack

This repository provides three ways to run the robot stack:

1. Real hardware
2. Gazebo simulation
3. Isaac Sim simulation

Each of the launch files below starts the complete robot stack for its respective environment, including robot control, motion planning, and supporting services. In most cases, these are the only launch files required to operate the system.

Before launching any configuration, source the workspace:

```bash
cd ashr_ws
source install/setup.bash
```

## Full Stack Launch Options

### Gazebo Simulation

Launch the complete robot stack in Gazebo:

```bash
ros2 launch probot_bringup gazebo_bringup.launch.py
```

### Real Hardware

Launch the complete robot stack on the physical robot:

```bash
ros2 launch probot_bringup hardware_demo.launch.py
```

### Isaac Sim Simulation

Launch the complete robot stack connected to Isaac Sim:

```bash
ros2 launch probot_bringup probot_bringup.launch.py use_mock_hardware:=false use_isaac:=true use_sim_time:=true
```

## Standalone Utilities

The launch files above are intended to start the complete system. However, individual components can also be launched independently for development, testing, or debugging purposes.

### Joystick Teleoperation

Launch the Xbox controller interface for MoveIt Servo teleoperation:

```bash
ros2 launch teleoperation joystick_servo.launch.py
```

### Vision Pipeline

Launch the Kinect-based vision pipeline:

```bash
ros2 launch vision kinect.launch.py
```

These utility launch files can be used independently or alongside any of the full-stack launch configurations when additional functionality is required.

