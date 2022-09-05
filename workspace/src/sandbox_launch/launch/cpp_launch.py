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

    # image receiver
    image_receiver = Node(
        package="cpp_image_receiver",
        executable="image_receiver_node" 
    )
    ld.add_action(image_receiver)

    return ld
