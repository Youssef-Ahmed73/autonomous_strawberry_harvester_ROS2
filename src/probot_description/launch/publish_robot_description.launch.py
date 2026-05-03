from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('probot_description'),
        'urdf',
        'probot_anno.xacro'
    ])

    robot_description = Command([
        'xacro ',
        xacro_file
    ],on_stderr='warn')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str)
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str)
        }]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node
    ])
