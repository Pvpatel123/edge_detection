#!/usr/bin/env python3

import rospy
import os
import cv2
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse
from edge_detector import CannyEdgeDetector

class EdgeDetectionService:
    def __init__(self):
        """
        Intializes the Edge Detection Service.
        - Registers a ROS service named 'edge+detection'.
        - Uses the Canny Edge Detector to process images.
        """
        rospy.init_node('edge_detection_service')
        self.detector = CannyEdgeDetector()
        self.service = rospy.Service('edge_detection', EdgeDetection, self.handle_edge_detection)
        rospy.loginfo(" Canny Edge Detection Service is Ready.")

    def handle_edge_detection(self, req):
        """
        Service callback fucntion that processes the given image or directory.

        :param req: ROS service request containing the image path
        :return: EdgeDetectionResponse containing processed image path(s)
        """
        image_path = req.image_path

        # If the input is a directory, process all images inside
        if os.path.isdir(image_path):
            rospy.loginfo(f" Processing all images in directory: {image_path}")
            processed_images = []
            for file in os.listdir(image_path):
                if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                    full_path = os.path.join(image_path, file)
                    try:
                        processed_image = self.detector.detect_and_highlight_edges(full_path)
                        output_path = os.path.join(image_path, f"edges_{file}")
                        self.detector.save_image(processed_image, output_path)
                        processed_images.append(output_path)
                        rospy.loginfo(f"Processed: {output_path}")
                    except Exception as e:
                        rospy.logerr(f" Error processing {file}: {e}")
            return EdgeDetectionResponse(";".join(processed_images))

        # If the input is a single image
        elif os.path.exists(image_path):
            try:
                processed_image = self.detector.detect_and_highlight_edges(image_path)
                output_path = image_path.replace(".jpg", "_edges.jpg").replace(".png", "_edges.png")
                self.detector.save_image(processed_image, output_path)
                rospy.loginfo(f" Processed Image Saved: {output_path}")
                return EdgeDetectionResponse(output_path)
            except Exception as e:
                rospy.logerr(f" Error processing image: {e}")
                return EdgeDetectionResponse("")
        else:
            rospy.logwarn(f"File or directory not found: {image_path}")
            return EdgeDetectionResponse("")

if __name__ == "__main__":
    EdgeDetectionService()
    rospy.spin()
