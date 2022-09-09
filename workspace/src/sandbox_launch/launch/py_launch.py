from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # image spawner
    image_spawner = Node(
        package="image_spawner",
        executable="image_spawner_node" 
    )
    ld.add_action(image_spawner)

    # image processor
    image_processor = Node(
        package="py_image_processor",
        executable="image_processor_node" 
    )
    ld.add_action(image_processor)

    # image receiver
    image_receiver = Node(
        package="py_image_receiver",
        executable="image_receiver_node" 
    )
    ld.add_action(image_receiver)

    return ld
