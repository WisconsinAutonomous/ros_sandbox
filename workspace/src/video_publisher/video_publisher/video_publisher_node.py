import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from wauto_perception_msgs.msg import (RoiArray, Roi, ObjectClassification)
from ament_index_python.packages import get_package_share_directory
import math
import os
import cv2
import numpy as np

class VideoPublisher(Node):

    def __init__(self):
        super().__init__('video_publisher')
        self.image_publisher = self.create_publisher(Image, "~/output/image", 1)
        self.bridge = CvBridge()

        self.declare_parameter('fps', 10)
        # fps to play the video back at
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        timer_period = 1/self.fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('package_path', '')
        self.declare_parameter('video_path', '')
        self.video_capture = None

    def timer_callback(self):
        if self.video_capture is None:
            self.video_capture = cv2.VideoCapture(os.path.join(self.get_parameter('package_path').get_parameter_value().string_value,
                                    self.get_parameter('video_path').get_parameter_value().string_value))
        
        
        if (self.video_capture.isOpened()):
            ret, frame = self.video_capture.read()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            image_msg.header.stamp = self.get_time_msg()
            image_msg.header.frame_id = "sample traffic light image"

            print("Published!")
            self.image_publisher.publish(image_msg)
        else:
            print("Done!")

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])

        return time_msg


def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    rclpy.spin(video_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()