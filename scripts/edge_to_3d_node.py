#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class EdgeTo3DNode:
    def __init__(self):
        """
        Initializes the Edge-to-3d Node.
        - Subcribers to edge-detected images, depth images, and camera info.
        - converts edge pixel to 3D coordinates using depth information.
        - Publishes the resulting 3D points as a PointCloud message.

        """
        rospy.init_node("edge_to_3d_node", anonymous=True)

        # Load parameters
        self.image_topic = rospy.get_param("~image_topic", "/edge_detection/output")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/edge_points")
        self.cam_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")

        

        self.bridge = CvBridge()

        # Subscribers
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        rospy.Subscriber(self.cam_info_topic, CameraInfo , self.cam_info_callback)

        # Publisher
        self.pointcloud_pub = rospy.Publisher(self.pointcloud_topic, PointCloud, queue_size=10)

        self.latest_depth_image = None  # Store the latest depth image

    def depth_callback(self, msg):
        """ 
        Store the latest depth image.
        This is used later for computing the 3D coordinates of edge pixels. 
        """
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except CvBridgeError as e:
            rospy.logerr(f"❌ CvBridge Error: {e}")

    def cam_info_callback(self, msg):
        """ Get the intrinsic camera matrix """
        self.intrinsic_matrix = msg.K

        self.fx = self.intrinsic_matrix[0]
        self.fy = self.intrinsic_matrix[4]
        self.cx = self.intrinsic_matrix[2]
        self.cy = self.intrinsic_matrix[5]
        self.scale = 1000.0

    def image_callback(self, msg):
        """ Convert edge pixels (u, v) to 3D coordinates (x, y, z) """
        if self.latest_depth_image is None:
            rospy.logwarn("⚠️ No depth image received yet!")
            return

        try:
            # Convert edge image to OpenCV format
            edge_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            gray = cv2.cvtColor(edge_image, cv2.COLOR_BGR2GRAY)
            
            # Get edge pixel locations
            edge_pixels = np.column_stack(np.where(gray > 0))
            #import ipdb; ipdb.set_trace()

            # Convert to 3D Points
            points = []
            for (v, u) in edge_pixels:
                depth = self.latest_depth_image[v, u] / self.scale  # Convert to meters
                if depth == 0:
                    continue  # Ignore invalid depth

                # Convert pixel (u, v) to camera coordinates (x, y, z)
                x = (u - self.cx) * depth / self.fx
                y = (v - self.cy) * depth / self.fy
                z = depth

                points.append(Point32(x, y, z))

            # Publish PointCloud message
            self.publish_pointcloud(points, msg.header)

        except CvBridgeError as e:
            rospy.logerr(f"❌ CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"❌ Error processing edge-to-3D conversion: {e}")

    def publish_pointcloud(self, points, header):
        """ Publish the 3D points as a PointCloud message """
        if len(points) == 0:
            rospy.logwarn("⚠️ No valid 3D points to publish!")
            return

        cloud_msg = PointCloud()
        cloud_msg.header = header
        cloud_msg.header.frame_id = "camera_color_optical_frame"
        cloud_msg.points = points

        self.pointcloud_pub.publish(cloud_msg)

        rospy.loginfo(f"✅ Published {len(points)} edge points to {self.pointcloud_topic}")

if __name__ == "__main__":
    EdgeTo3DNode()
    rospy.spin()
