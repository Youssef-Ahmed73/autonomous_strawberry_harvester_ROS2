# WELCOME TO ASHR GRADUATION PROJECT

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