from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # Define your camera URLs here
    camera1_url = 'http://10.42.0.171/stream'
    camera2_url = 'http://10.42.0.161/stream' # Replace with your actual second IP camera URL

    # ---------------------------------------------------------
    # CONTAINER 1: Camera 1 & Inference 1
    # ---------------------------------------------------------
    container1 = ComposableNodeContainer(
        name='vision_container_1',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # Using multi-threaded container
        composable_node_descriptions=[
            ComposableNode(
                package='vision',
                plugin='vision::CameraComponent',
                name='camera_node',
                namespace='cam1', # Isolates topics to /cam1/image_raw
                parameters=[{
                    'stream_url': camera1_url,
                    'camera_name': 'cam_1'
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='vision',
                plugin='vision::InferenceComponent',
                name='inference_node',
                namespace='cam1', # Subscribes to /cam1/image_raw
                parameters=[{'debug_viz': True}], 
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # CONTAINER 2: Camera 2 & Inference 2
    # ---------------------------------------------------------
    container2 = ComposableNodeContainer(
        name='vision_container_2',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='vision',
                plugin='vision::CameraComponent',
                name='camera_node',
                namespace='cam2', # Isolates topics to /cam2/image_raw
                parameters=[{
                    'stream_url': camera2_url,
                    'camera_name': 'cam_2'
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='vision',
                plugin='vision::InferenceComponent',
                name='inference_node',
                namespace='cam2', # Subscribes to /cam2/image_raw
                parameters=[{'debug_viz': True}], 
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container1,
        container2
    ])