import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.logger = rclpy.logging.get_logger(self.get_name())

        self.image_sub = self.create_subscription(Image, "image/original", self.image_callback, 10)
        self.image_sub # avoid unused variable warning

        self.image_pub = self.create_publisher(Image, "image/grayscale", 1)

        self.clock_ = rclpy.clock.Clock()
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cur_time = self.clock_.now()
        secs = cur_time.nanoseconds//1000000000 - msg.header.stamp.sec
        nsecs = cur_time.nanoseconds%1000000000 - msg.header.stamp.nanosec
        self.get_logger().info(f"Time diff: {secs}.{nsecs}s")

        cv_image = self.bridge.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)       
        gray_msg = self.bridge.cv2_to_imgmsg(gray)

        gray_msg.header.stamp = self.clock_.now().to_msg()
        self.image_pub.publish(gray_msg)
        

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()

    rclpy.spin(image_processor_node)

    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
