#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class EdgeDetectionNode:
    def __init__(self):
        """
        Initializes the Edge Detection Node.
        - Subcribes to an image topic.
        - Applies edge detection to incoming images.
        - Publshes the processed images to another topic.
        """
        rospy.init_node("edge_detection_node", anonymous=True)

        # Choose the correct image topic from available topics
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")  
        self.edge_topic = rospy.get_param("~edge_topic", "/edge_detection/output")

        self.bridge = CvBridge()
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.edge_pub = rospy.Publisher(self.edge_topic, Image, queue_size=10)

        rospy.loginfo(f" Subscribed to {self.image_topic}")
        rospy.loginfo(f" Publishing edges to {self.edge_topic}")

    def image_callback(self, msg):
        """
        Callback fucntion for image processing.
        - Converts the ROS Image message to an OpenCV image.
        - Applies Canny edge detection.
        - Highlights the edges in green.
        - Publishes the processed image.

        :param msg: ROS Image message
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            
            edge_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            edge_colored[edges > 0] = [0, 255, 0]  # Green edges
            
            edge_msg = self.bridge.cv2_to_imgmsg(edge_colored, "bgr8")
            self.edge_pub.publish(edge_msg)

            rospy.loginfo(" Edge detection image published")
        
        except CvBridgeError as e:
            rospy.logerr(f" CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f" Error processing image: {e}")

if __name__ == "__main__":
    EdgeDetectionNode()
    rospy.spin()
