#!/usr/bin/env python3

import rospy
import os
import sys
from edge_detection.srv import EdgeDetection

def call_edge_detection(image_path):
    rospy.wait_for_service('edge_detection')
    try:
        edge_detection = rospy.ServiceProxy('edge_detection', EdgeDetection)
        response = edge_detection(image_path)
        if response.processed_image_path:
            rospy.loginfo(f"Processed Image: {response.processed_image_path}")
        else:
            rospy.logwarn("Processing failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('edge_detection_client')

    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun edge_detection edge_detection_client.py <image_directory>")
        sys.exit(1)

    directory = sys.argv[1]

    if not os.path.isdir(directory):
        rospy.logerr(f"Invalid directory: {directory}")
        sys.exit(1)

    images = [f for f in os.listdir(directory) if f.endswith(('.jpg', '.png'))]

    for image in images:
        image_path = os.path.join(directory, image)
        call_edge_detection(image_path)
