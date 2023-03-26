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

class MultiImagePublisher(Node):

    def __init__(self):
        super().__init__('multi_image_publisher')
        # get the package path
        self.declare_parameter('package_path', '')
        self.package_path = self.get_parameter('package_path').value

        # get image paths
        self.declare_parameter('image_paths', ["data/image.jpg"])
        self.image_paths = self.get_parameter('image_paths').value

        # get image topics
        self.declare_parameter('image_topics', ["/sensing/cc/raw/image"])
        self.image_topics = self.get_parameter('image_topics').value

        self.image_publishers = []
        self.images = []
        for i in range(len(self.image_paths)):
            self.image_publishers.append(self.create_publisher(Image, self.image_topics[i], 1))

            img_path = os.path.join(self.package_path, self.image_paths[i])
            print(img_path)
            img = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_RGB2BGR)
            self.images.append(img)
    
        self.bridge = CvBridge()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        stamp = self.get_time_msg()
        for i in range(len(self.image_paths)):
            image_msg = self.bridge.cv2_to_imgmsg(self.images[i], encoding="rgb8")
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = f"image_{i}"
            self.image_publishers[i].publish(image_msg)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])

        return time_msg


def main(args=None):
    rclpy.init(args=args)

    multi_image_publisher = MultiImagePublisher()

    rclpy.spin(multi_image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()