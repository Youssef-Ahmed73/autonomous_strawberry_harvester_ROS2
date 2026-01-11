cmake_minimum_required(VERSION 3.8)
project(probot_description)

# Required for ROS 2
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_ros REQUIRED)

#define dir
install(
  DIRECTORY launch urdf meshes 
  DESTINATION share/${PROJECT_NAME}
)

#define files
install(
  FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
