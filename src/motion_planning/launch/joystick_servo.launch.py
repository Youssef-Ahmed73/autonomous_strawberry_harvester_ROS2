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
        get_package_share_directory("motion_planning"),
        "config",
        "servo_config.yaml",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{
            "use_sim_time": False,
            "autorepeat_rate": 20.0  # FIX: Forces joy to publish 20 times/sec even when holding the stick still
        }],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",  # FIX: Ensures private topics resolve to /servo_node/...
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'moveit_servo:=DEBUG', '--log-level', 'pick_ik:=DEBUG']
    )

    joystick_servo_node = Node(
        package="motion_planning",
        executable="joystick_servo_node",
        name="joystick_servo_node",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    return LaunchDescription([joy_node, servo_node, joystick_servo_node])