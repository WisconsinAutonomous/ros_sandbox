from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='image_roi_publisher',
            namespace='image_roi_publisher',
            executable='image_roi_publisher_node',
            name='image_roi_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"package_path":    "/home/ros_sandbox/ros_sandbox/workspace/src/image_roi_publisher",
                 "image_path":      "data/lane_lines/mcity.jpg",                  
                 "roi_path":        "data/lane_lines/mcity.txt"} 
            ],
            # remappings=[
            #     ("~/output/image")
            # ]
        ),
    ])