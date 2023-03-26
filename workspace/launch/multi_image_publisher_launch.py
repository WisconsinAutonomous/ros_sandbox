from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='multi_image_publisher',
            namespace='multi_image_publisher',
            executable='multi_image_publisher_node',
            name='multi_image_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"package_path":    "/home/ros_sandbox/ros_sandbox/workspace/src/multi_image_publisher",
                 "image_paths":     ["data/mcity/image1.jpg",
                                     "data/mcity/image2.jpg",
                                     "data/mcity/image3.jpg"],
                 "image_topics":    ["/sensing/cc/raw/image",
                                     "/sensing/lc/raw/image",
                                     "/sensing/rc/raw/image"]
                } 
            ],
            # remappings=[
            #     ("~/output/image")
            # ]
        ),
    ])