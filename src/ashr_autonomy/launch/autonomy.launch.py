import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. File Paths
    autonomy_pkg_dir = get_package_share_directory('ashr_autonomy')
    autonomy_params_file = os.path.join(autonomy_pkg_dir, 'config', 'autonomy_params.yaml')
    pilz_params_file = os.path.join(autonomy_pkg_dir, 'config', 'moveit_pilz_config.yaml')

    # 2. Minimal MoveIt Builder
    moveit_config = (
        MoveItConfigsBuilder("probot_anno", package_name="moveit_config")
        .robot_description(
            file_path="config/probot_anno.urdf.xacro", 
            mappings={
                "use_gazebo": "true",
                "use_mock_hardware": "false",
            }
        )
        .to_moveit_configs()
    )

    # 3. Nodes
    target_server_node = Node(
        package='ashr_autonomy',
        executable='target_server',
        name='target_server_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    autonomy_fsm_node = Node(
        package='ashr_autonomy',
        executable='autonomy_node',
        name='autonomy_fsm_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            pilz_params_file,          
            autonomy_params_file,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        target_server_node,
        autonomy_fsm_node
    ])