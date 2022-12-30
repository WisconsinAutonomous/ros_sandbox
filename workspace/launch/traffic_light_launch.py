from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='traffic_light_spawner',
            namespace='traffic_light_spawner',
            executable='traffic_light_spawner_node',
            name='traffic_light_spawner',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"package_path":    "/home/ros_sandbox/ros_sandbox/workspace/src/traffic_light_spawner",
                 "image_path":      "data/sample1.jpg",                  
                 "roi_path":            "data/sample1.txt"} 
            ],
            # remappings=[
            #     ("~/output/image")
            # ]
        ),
    ])