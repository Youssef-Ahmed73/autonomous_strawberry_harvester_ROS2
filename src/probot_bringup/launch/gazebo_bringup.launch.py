import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path to your main bringup launch file
    pkg_probot_bringup = get_package_share_directory('probot_bringup')
    main_bringup_launch_file = os.path.join(pkg_probot_bringup, 'launch', 'probot_bringup.launch.py')

    # 2. Call the main launch file with the specific arguments you requested
    probot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_bringup_launch_file),
        launch_arguments={
            'use_mock_hardware': 'false',
            'use_real_hardware': 'false',
            'use_gazebo': 'true',
            'use_isaac': 'false',
            'use_sim_time': 'true',
        }.items()
    )

    # 3. Add the Vision Pipeline Components
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # --------------------------------------------------
            # Main Camera (Kinect) - Inference & Target Locator
            # --------------------------------------------------
            ComposableNode(
                package='vision',
                plugin='vision::InferenceComponent',
                name='inference_node',
                parameters=[
                    {'debug_viz': True},
                    {'use_sim_time': True}
                ],
                remappings=[
                    # Map the inference input to Gazebo's bridged Kinect RGB topic
                    ('image_raw', '/camera/color/image_raw')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='vision',
                plugin='vision::TargetLocatorComponent',
                name='target_locator_node',
                parameters=[
                    {'use_sim_time': True}
                ],
                remappings=[
                    # Map the locator inputs to Gazebo's bridged Kinect Depth & Info topics
                    ('kinect/depth/image_raw', '/camera/depth/image_raw'),
                    ('kinect/depth/camera_info', '/camera/depth/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # --------------------------------------------------
            # ESP Camera 1 - Classical Vision Pipeline
            # --------------------------------------------------
            ComposableNode(
                package='vision',
                plugin='vision::ClassicalVisionComponent',
                name='classical_vision_esp1',
                parameters=[
                    {'debug_viz': True},
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('image_raw', '/camera/esp1/image_raw'),
                    # Remap outputs to prevent cross-talk with Kinect and ESP2
                    ('detections', 'esp1/detections'),
                    ('detections_debug', 'esp1/detections_debug'),
                    ('/verify_ripeness', '/esp1/verify_ripeness') 
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # --------------------------------------------------
            # ESP Camera 2 - Classical Vision Pipeline
            # --------------------------------------------------
            ComposableNode(
                package='vision',
                plugin='vision::ClassicalVisionComponent',
                name='classical_vision_esp2',
                parameters=[
                    {'debug_viz': True},
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('image_raw', '/camera/esp2/image_raw'),
                    # Remap outputs to prevent cross-talk with Kinect and ESP1
                    ('detections', 'esp2/detections'),
                    ('detections_debug', 'esp2/detections_debug'),
                    ('/verify_ripeness', '/esp2/verify_ripeness')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        probot_bringup_launch,
        vision_container
    ])