import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("probot_anno", package_name="moveit_config")
        .robot_description(file_path="config/probot_anno.urdf.xacro")
        .to_moveit_configs()
    )

    servo_yaml = os.path.join(
        get_package_share_directory('moveit_config'),
        'config',
        'probot_servo_config.yaml'
    )
    
    joy_yaml = os.path.join(
        get_package_share_directory('moveit_config'),
        'config',
        'probot_joy_config.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': True}]
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                joy_yaml,
                {
                    'use_sim_time': True,
                    'publish_stamped_twist': True,
                    'frame': 'base_link'
                }
            ],
            remappings=[
                ('/cmd_vel', '/servo_node/delta_twist_cmds')
            ],
            output='screen'
    )


    return LaunchDescription([
        joy_node,
        servo_node,
        teleop_twist_joy_node,
    ])