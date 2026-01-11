import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('probot_description')

    # 1. Path to your existing description launch file
    description_launch_path = os.path.join(pkg_path, 'launch', 'publish_robot_description.launch.py') 

    # 2. Path to your RViz config file
    # If you moved it to an 'rviz' folder, change 'urdf.rviz' to 'rviz/urdf.rviz'
    rviz_config_path = os.path.join(pkg_path, 'urdf.rviz')

    # Include the robot_state_publisher launch
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        launch_description,
        rviz_node
    ])