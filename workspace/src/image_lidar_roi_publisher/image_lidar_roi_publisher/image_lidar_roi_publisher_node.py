import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs.msg
import std_msgs.msg
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from wauto_perception_msgs.msg import (RoiArray, Roi, ObjectClassification)
from ament_index_python.packages import get_package_share_directory
import math
import os
import cv2
import numpy as np
import pandas as pd

class ImageLidarROIPublisher(Node):

    def __init__(self):
        super().__init__('image_lidar_roi_publisher')
        self.image_publisher = self.create_publisher(sensor_msgs.msg.Image, "~/output/image", 1)
        self.roi_publisher = self.create_publisher(RoiArray, "~/output/rois", 1)
        self.pc_publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "~/output/pts", 1)
        self.bridge = CvBridge()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('package_path', '')
        self.declare_parameter('image_path', '')
        self.declare_parameter('roi_path', '')
        self.declare_parameter('pc_path', '')
        self.image = None
        self.rois = None
        self.pc = None

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

        if self.pc is None:
            pc_path = os.path.join(self.get_parameter('package_path').get_parameter_value().string_value,
                                    self.get_parameter('pc_path').get_parameter_value().string_value)
            print(pc_path)
            lidar = pd.read_pickle(pc_path)
            lidar_points = np.concatenate((
                lidar["x"].reshape(-1, 1),
                lidar["y"].reshape(-1, 1),
                lidar["z"].reshape(-1, 1),
                lidar["i"].reshape(-1, 1)
            ))

            self.pc = self.point_cloud(lidar_points, "lidar_frame")

        roi_array_msg = RoiArray()
        for i,roi in enumerate(self.rois):
            cx = roi[1] * self.image.shape[1]
            cy = roi[2] * self.image.shape[0]
            w = roi[3] * self.image.shape[1]
            h = roi[4] * self.image.shape[0]
            roi_msg = Roi()
            roi_msg.id = i
            
            # CHANGE THIS DEPENDING ON THE CLASS OF OBJECTS
            roi_msg.classification.classification = ObjectClassification.WA_OBJECT_CLASSIFICATION_TRAFFIC_SIGN
            
            roi_msg.bottom_left.x = cx-w/2
            roi_msg.bottom_left.y = cy+h/2
            roi_msg.top_right.x = cx+w/2
            roi_msg.top_right.y = cy-h/2
        
            roi_array_msg.rois.append(roi_msg)
        
        roi_array_msg.header.stamp = self.get_time_msg()
        roi_array_msg.header.frame_id = "sample traffic light bounding boxes"

        pc_msg = self.pc
        pc_msg.header.stamp = self.get_time_msg()

        self.image_publisher.publish(image_msg)
        self.roi_publisher.publish(roi_array_msg)
        self.pc_publisher.publish(pc_msg)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])

        return time_msg

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx4 array of xyz positions (m) and intensity i
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = sensor_msgs.msg.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [sensor_msgs.msg.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzi')]

        header = std_msgs.msg.Header(frame_id=parent_frame, stamp=self.get_time_msg())

        return sensor_msgs.msg.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 4),
            row_step=(itemsize * 4 * points.shape[0]),
            data=data
        )


def main(args=None):
    rclpy.init(args=args)

    image_lidar_roi_publisher = ImageLidarROIPublisher()

    rclpy.spin(image_lidar_roi_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_roi_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()