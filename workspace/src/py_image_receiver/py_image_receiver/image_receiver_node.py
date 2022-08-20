import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import math

class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__('image_receiver_node')
        self.image_subscription = self.create_subscription(Image, "image", self.image_received_callback, 10)
        self.clock_ = rclpy.clock.Clock()
        self.image_subscription

    def image_received_callback(self, msg):
        cur_time = self.clock_.now()
        secs = cur_time.nanoseconds//1000000000 - msg.header.stamp.sec
        nsecs = cur_time.nanoseconds%1000000000 - msg.header.stamp.nanosec
        self.get_logger().info(f"Image Received!\tTime diff: {secs}.{nsecs}\tseconds")

def main(args=None):
    rclpy.init(args=args)
    image_receiver_node = ImageReceiverNode()

    rclpy.spin(image_receiver_node)

    image_receiver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
