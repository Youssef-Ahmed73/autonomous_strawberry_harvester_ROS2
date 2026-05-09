# WELCOME TO ASHR GRADUATION PROJECT 🍓

## About the project
I will write this later XD

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