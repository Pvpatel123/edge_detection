#!/usr/bin/env python3

import rospy
import cv2
import os
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse
from cv_bridge import CvBridge

class EdgeDetectionService:
    def __init__(self):
        rospy.init_node('edge_detection_service')
        self.bridge = CvBridge()
        self.service = rospy.Service('edge_detection', EdgeDetection, self.handle_edge_detection)
        rospy.loginfo("Edge Detection Service is Ready.")

    def handle_edge_detection(self, req):
        input_path = req.image_path
        if not os.path.exists(input_path):
            rospy.logwarn(f"File does not exist: {input_path}")
            return EdgeDetectionResponse("")

        # Load image and apply edge detection
        image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
        edges = cv2.Canny(image, 50, 150)

        # Save the processed image
        output_path = input_path.replace(".jpg", "_edges.jpg").replace(".png", "_edges.png")
        cv2.imwrite(output_path, edges)

        rospy.loginfo(f"Processed image saved: {output_path}")
        return EdgeDetectionResponse(output_path)

if __name__ == "__main__":
    EdgeDetectionService()
    rospy.spin()
