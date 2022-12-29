import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from wauto_perception_msgs.msg import (RoiArray, Roi, ObjectClassification)
import math

class TrafficLightSpawner(Node):

    def __init__(self):
        super().__init__('traffic_light_spawner')
        self.image_publisher = self.create_publisher(Image, "~/output/image", 1)
        self.roi_publisher = self.create_publisher(RoiArray, "~/output/rois", 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.image_path = ""
        self.image = None

    def timer_callback(self):
        if self.image is None:
            self.image = cv2.cvtColor(cv2.imread(self.image_path), cv2.COLOR_RGB2BGR)
        
        image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="rgb8")
        image_msg.header.stamp = self.get_time_msg()
        image_msg.header.frame_id = "sample traffic light image"
        self.image_publisher.publish(image_msg)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])

        return time_msg


def main(args=None):
    rclpy.init(args=args)

    traffic_light_spawner = TrafficLightSpawner()

    rclpy.spin(traffic_light_spawner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traffic_light_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()