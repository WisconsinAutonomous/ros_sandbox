from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='image_lidar_roi_publisher',
            namespace='image_lidar_roi_publisher',
            executable='image_lidar_roi_publisher_node',
            name='image_lidar_roi_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"package_path":    "/home/ros_sandbox/ros_sandbox/workspace/src/image_lidar_roi_publisher",
                 "image_path":      "data/00002714.jpg",                  
                 "roi_path":        "data/00002714.txt",
                 "pc_path":         "data/00002714.pkl"} 
            ],
            # remappings=[
            #     ("~/output/image")
            # ]
        ),
    ])