# WELCOME TO ASHR GRADUATION PROJECT

## About the project
I will write this later XD

## Installing the repo
```bash
mkdir ashr_ws && cd ashr_ws
git clone https://github.com/Youssef-Ahmed73/autonomous_strawberry_harvester_ROS2
colcon build
```
## Running simulation (gazebo)
after building the work space open terminal and run the following commads to run gazebo

```bash
source install/setup.bash
ros2 launch probot_gazebo sim.launch.py
```
then open another terminal to open rviz to control robot and run the following commands
```bash
source install/setup.bash
ros2 launch probot_gazebo moveit_gazebo.launch.py use_sim_time:=true
```