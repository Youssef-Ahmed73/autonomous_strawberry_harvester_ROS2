from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='vision_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='vision',
                    plugin='vision::CameraComponent',
                    name='camera_node',
                    #parameters=[{'stream_url': 'http://10.42.0.171/stream'}], #camera 1
                    parameters=[{'stream_url': 'http://10.42.0.161/stream'}],  #camera 2
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='vision',
                    plugin='vision::InferenceComponent',
                    name='inference_node',
                    parameters=[{'debug_viz': True}], 
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        )
    ])