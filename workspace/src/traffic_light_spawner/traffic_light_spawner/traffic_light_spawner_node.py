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

class TrafficLightSpawner(Node):

    def __init__(self):
        super().__init__('traffic_light_spawner')
        self.image_publisher = self.create_publisher(Image, "~/output/image", 1)
        self.roi_publisher = self.create_publisher(RoiArray, "~/output/rois", 1)
        self.bridge = CvBridge()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('package_path', '')
        self.declare_parameter('image_path', '')
        self.declare_parameter('roi_path', '')
        self.image = None
        self.rois = None

    def timer_callback(self):
        if self.image is None:
            img_path = os.path.join(self.get_parameter('package_path').get_parameter_value().string_value,
                                    self.get_parameter('image_path').get_parameter_value().string_value)
            print(img_path)
            self.image = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_RGB2BGR)
        
        
        image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="rgb8")
        image_msg.header.stamp = self.get_time_msg()
        image_msg.header.frame_id = "sample traffic light image"

        if self.rois is None:
            roi_path = os.path.join(self.get_parameter('package_path').get_parameter_value().string_value,
                                    self.get_parameter('roi_path').get_parameter_value().string_value)
            

            self.rois = np.loadtxt(roi_path)

        roi_array_msg = RoiArray()
        for i,roi in enumerate(self.rois):
            cx = roi[1] * self.image.shape[1]
            cy = roi[2] * self.image.shape[0]
            w = roi[3] * self.image.shape[1]
            h = roi[4] * self.image.shape[0]
            roi_msg = Roi()
            roi_msg.id = i
            roi_msg.classification.classification = ObjectClassification.WA_OBJECT_CLASSIFICATION_TRAFFIC_LIGHT
            roi_msg.bottom_left.x = cx-w/2
            roi_msg.bottom_left.y = cy+h/2
            roi_msg.top_right.x = cx+w/2
            roi_msg.top_right.y = cy-h/2
        
            roi_array_msg.rois.append(roi_msg)
        
        roi_array_msg.header.stamp = self.get_time_msg()
        roi_array_msg.header.frame_id = "sample traffic light bounding boxes"

        self.image_publisher.publish(image_msg)
        self.roi_publisher.publish(roi_array_msg)

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