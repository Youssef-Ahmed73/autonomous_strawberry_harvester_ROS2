import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # =========================================================
    # Zero-Copy Vision Pipeline Container
    # =========================================================
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # Multi-threaded container for heavy image processing
        composable_node_descriptions=[
            
            # 1. Hardware Driver (The Real Kinect)
            ComposableNode(
                package='vision',
                plugin='vision::KinectComponent',
                name='kinect_node',
                parameters=[
                    {'use_sim_time': False}
                ],
                # If your KinectComponent publishes to different topics than what Inference/Locator expect, 
                # you can add remappings here. Otherwise, they will auto-connect!
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # 2. AI Inference (YOLO / ONNX)
            ComposableNode(
                package='vision',
                plugin='vision::InferenceComponent',
                name='inference_node',
                parameters=[
                    {'debug_viz': True},
                    {'use_sim_time': False}
                ],
                remappings=[
                    # UPDATE THESE to match your real Kinect's topics from 'ros2 topic list'
                    ('image_raw', '/kinect/rgb/image_raw') 
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # 3. 3D Target Locator (Pinhole Math & Depth Extraction)
            ComposableNode(
                package='vision',
                plugin='vision::TargetLocatorComponent',
                name='target_locator_node',
                parameters=[
                    {'use_sim_time': False}
                ],
                remappings=[
                    # UPDATE THESE: The locator needs BOTH the depth image and the camera info!
                    ('kinect/depth/image_raw', '/kinect/depth/image_raw'),
                    ('kinect/depth/camera_info', '/kinect/depth/camera_info') 
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        vision_container
    ])