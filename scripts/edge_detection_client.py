#!/usr/bin/env python3

import rospy
import sys
import os
from edge_detection.srv import EdgeDetection

def call_edge_detection_service(image_path):
    """

    Fucntion to call ROS edge detection service.
    It sends the image path to the service and receives the processed image path(s).

    :para image_path: Path of the input image or directory containing images

    """
    
    rospy.wait_for_service('edge_detection')
    try:
        edge_detection = rospy.ServiceProxy('edge_detection', EdgeDetection)
        response = edge_detection(image_path)

        if response.processed_image_path:
            processed_files = response.processed_image_path.split(";")
            for file in processed_files:
                rospy.loginfo(f"Processed Image: {file}")
        else:
            rospy.logwarn("Edge detection failed.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: rosrun edge_detection edge_detection_client.py <image_or_directory_path>")
        sys.exit(1)

    image_path = sys.argv[1]

    if not os.path.exists(image_path):
        print(f"Error: Path does not exist -> {image_path}")
        sys.exit(1)

    call_edge_detection_service(image_path)
