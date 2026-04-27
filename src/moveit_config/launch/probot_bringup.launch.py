import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. Packages & Paths
    # ---------------------------------------------------------
    pkg_probot_description = get_package_share_directory('probot_description')
    pkg_moveit_config = get_package_share_directory('moveit_config') 
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    controllers_yaml = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')
    rviz_config_file = os.path.join(pkg_moveit_config, 'config', 'moveit.rviz')

    # ---------------------------------------------------------
    # 2. Launch Configurations
    # ---------------------------------------------------------
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_isaac = LaunchConfiguration('use_isaac')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------------------------------------------------
    # 3. MoveIt Config Builder
    # ---------------------------------------------------------
    # We pass the launch configurations as mappings directly into your Master Assembler
    moveit_config = (
        MoveItConfigsBuilder("probot_anno", package_name="moveit_config")
        .robot_description(
            file_path="config/probot_anno.urdf.xacro",
            mappings={
                "use_mock_hardware": use_mock_hardware,
                "use_real_hardware": use_real_hardware,
                "use_gazebo": use_gazebo,
                "use_isaac": use_isaac,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # ---------------------------------------------------------
    # 4. Core Nodes (Always Running)
    # ---------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---------------------------------------------------------
    # 5. Hardware/Control Nodes (Conditional)
    # ---------------------------------------------------------
    # Gazebo loads the controller_manager inside its physics plugin. 
    # For real hardware, mock hardware, or external sims, we must launch it explicitly.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers_yaml],
        output="screen",
        condition=UnlessCondition(use_gazebo), 
    )

    # ---------------------------------------------------------
    # 6. Gazebo (Ignition) Nodes (Conditional)
    # ---------------------------------------------------------
    install_dir = os.path.join(pkg_probot_description, '..')
    gz_resource_path = install_dir
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        condition=IfCondition(use_gazebo)
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'probot_anno', '-z', '0.1'],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    # ---------------------------------------------------------
    # 7. Controller Spawners
    # ---------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )


    # ---------------------------------------------------------
    # Return Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_mock_hardware', default_value='true', description='Use mock hardware'),
        DeclareLaunchArgument('use_real_hardware', default_value='false', description='Use real hardware'),
        DeclareLaunchArgument('use_gazebo', default_value='false', description='Use Gazebo simulation'),
        DeclareLaunchArgument('use_isaac', default_value='false', description='Use Isaac Sim'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gz_resource_path),

        # Standard ROS 2 Nodes
        robot_state_publisher,
        ros2_control_node,
        move_group_node,
        rviz_node,

        # Gazebo Nodes
        gazebo,
        spawn_entity,
        gz_bridge,

        # Controller Spawners
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])