import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='image_spawner',
                        plugin='image_spawner::image_spawner_node',
                        name='image_spawner_node',
                        ),
                    ComposableNode(
                        package='image_receiver',
                        plugin='image_receiver::image_receiver_node',
                        name='image_receiver_node',
                        )
                ],
            output='both',
    )

    return launch.LaunchDescription([container])
