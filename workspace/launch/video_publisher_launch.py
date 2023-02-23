from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()
    publisher_node = Node(
            package='video_publisher',
            namespace='video_publisher',
            executable='video_publisher_node',
            name='video_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"package_path":    "/home/ros_sandbox/ros_sandbox/workspace/src/video_publisher",
                 "video_path":      "data/mcity_video.MOV",                  
                 "fps": 10} 
            ],
            # remappings=[
            #     ("~/output/image")
            # ]
        )

    bag_play = ExecuteProcess(cmd=["ros2", "bag", "record", "--all"], output='screen')

    ld.add_action(publisher_node)
    ld.add_action(bag_play)
    
    return ld