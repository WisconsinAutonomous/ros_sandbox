import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import math

class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__('image_receiver_node')
        self.image_sub = self.create_subscription(Image, "image/grayscale", self.image_callback, 10)
        self.clock_ = rclpy.clock.Clock()
        self.image_sub

    def image_callback(self, msg):
        cur_time = self.clock_.now()
        secs = cur_time.nanoseconds//1000000000 - msg.header.stamp.sec
        nsecs = cur_time.nanoseconds%1000000000 - msg.header.stamp.nanosec
        self.get_logger().info(f"Time diff: {secs}.{nsecs}s")

def main(args=None):
    rclpy.init(args=args)
    image_receiver_node = ImageReceiverNode()

    rclpy.spin(image_receiver_node)

    image_receiver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
