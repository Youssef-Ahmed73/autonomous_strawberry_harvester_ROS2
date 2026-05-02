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
                    #parameters=[{'stream_url': 'http://192.168.100.130:8080/video'}],
                    parameters=[{'stream_url': 'http://10.140.187.152:8080/video'}],
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