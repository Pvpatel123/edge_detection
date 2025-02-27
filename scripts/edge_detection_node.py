#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header

class EdgeDetectionNode:
    def __init__(self):
        rospy.init_node('edge_detection_node')

        self.bridge = CvBridge()

        # Subscribe to RGB and Depth image topics
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth", Image, self.depth_callback)

        # Publishers
        self.edge_image_pub = rospy.Publisher("/edge_detection/edges", Image, queue_size=10)
        self.pc_pub = rospy.Publisher("/edge_detection/edge_points", PointCloud2, queue_size=10)

        self.depth_image = None

        # Camera parameters (Modify these based on your calibration)
        self.fx = 525.0  # Focal length in x
        self.fy = 525.0  # Focal length in y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y

    def depth_callback(self, msg):
        # Convert depth image from ROS format to OpenCV
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert edges to ROS Image and publish
        edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
        self.edge_image_pub.publish(edge_msg)

        # Convert edge pixels to 3D points
        if self.depth_image is not None:
            edge_pixels = np.column_stack(np.where(edges > 0))
            points = []

            for u, v in edge_pixels:
                z = self.depth_image[v, u] * 0.001  # Convert depth from mm to meters
                if z > 0:  # Ignore invalid depth values
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    points.append((x, y, z))

            # Publish as PointCloud2
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_link"

            cloud_msg = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(cloud_msg)

if __name__ == "__main__":
    EdgeDetectionNode()
    rospy.spin()
