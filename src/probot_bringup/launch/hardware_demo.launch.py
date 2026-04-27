import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------------------------------------
    # 1. Launch Configurations for Micro-ROS
    # ---------------------------------------------------------
    mros_port = LaunchConfiguration('mros_port')
    mros_baudrate = LaunchConfiguration('mros_baudrate')

    # ---------------------------------------------------------
    # 2. Include the Master Bringup (Forced to Real Hardware)
    # ---------------------------------------------------------
    # Update 'moveit_config' if your probot_bringup.launch.py is located elsewhere
    pkg_moveit_config = get_package_share_directory('moveit_config')
    
    probot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'probot_bringup.launch.py')
        ),
        # Hardcode the hardware arguments so this file ALWAYS launches the real robot
        launch_arguments={
            'use_mock_hardware': 'false',
            'use_real_hardware': 'true',
            'use_gazebo': 'false',
            'use_isaac': 'false',
        }.items()
    )

    # ---------------------------------------------------------
    # 3. Micro-ROS Agent Node
    # ---------------------------------------------------------
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', mros_port, '-b', mros_baudrate]
    )

    # ---------------------------------------------------------
    # Return Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        # Declare arguments with standard defaults for embedded boards
        DeclareLaunchArgument(
            'mros_port', 
            default_value='/dev/ttyUSB0', 
            description='Serial port for the micro-ROS agent'
        ),
        DeclareLaunchArgument(
            'mros_baudrate', 
            default_value='115200', 
            description='Baud rate for the micro-ROS agent'
        ),
        
        # Start the nodes
        micro_ros_agent_node,
        probot_bringup_launch,
    ])