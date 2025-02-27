import cv2
import numpy as np
import os
import sys


class CannyEdgeDetector:
    def __init__(self, low_threshold=50, high_threshold=150, blur_ksize=(5, 5), blur_sigma=1.5):
        """
        Canny Edge Detector for any image using Gaussian Blur and Canny.
        """
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.blur_ksize = blur_ksize
        self.blur_sigma = blur_sigma

    def preprocess_image(self, image):
        """
        Converts the image to grayscale and applies Gaussian blur to remove noise.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, self.blur_ksize, self.blur_sigma)
        return blurred

    def detect_canny_edges(self, image):
        """
        Applies Canny edge detection with adaptive thresholding.
        """
        mean_intensity = np.mean(image)
        auto_low = max(30, int(0.66 * mean_intensity))
        auto_high = min(255, int(1.33 * mean_intensity))

        if auto_high <= auto_low:  # Ensure thresholds are valid
            auto_high = auto_low + 20  # Adjust high threshold dynamically

        edges = cv2.Canny(image, auto_low, auto_high)
        return edges

    def highlight_edges(self, original_image, edges):
        """
        Superimposes detected edges onto the original image in green.
        """
        output = original_image.copy()
        output[edges > 0] = [0, 255, 0]  # Green edges
        return output

    def detect_and_highlight_edges(self, image_path):
        """
        Detects and highlights the edges of any image.
        """
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Error: File not found -> {image_path}")

        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("Error: Could not read the image. Check the file format.")

        preprocessed = self.preprocess_image(image)
        canny_edges = self.detect_canny_edges(preprocessed)

        highlighted_image = self.highlight_edges(image, canny_edges)
        return highlighted_image

    def save_image(self, image, output_path):
        """
        Saves the processed image to a file.
        """
        cv2.imwrite(output_path, image)
        print(f"✅ Image saved at: {output_path}")

    def show_image(self, image):
        """
        Displays the image and allows exiting by pressing 'q' or 'ESC'.
        """
        max_size = 800
        h, w = image.shape[:2]

        if max(h, w) > max_size:
            scale = max_size / max(h, w)
            image = cv2.resize(image, (int(w * scale), int(h * scale)))

        cv2.imshow("Canny Edge Detection Result", image)

        # Close the window when 'q' or 'ESC' is pressed
        key = cv2.waitKey(0) & 0xFF
        if key == 27 or key == ord('q'):  # ESC or 'q' to exit
            cv2.destroyAllWindows()

        cv2.destroyAllWindows()
        cv2.waitKey(1)
        sys.exit("✅ Program finished successfully.")  # Exit script properly


if __name__ == "__main__":
    # Ask the user for the image path
    image_path = input("Enter the full image path: ").strip()

    try:
        if not os.path.exists(image_path):
            print(f"Error: File does not exist -> {image_path}")
            sys.exit(1)

        detector = CannyEdgeDetector()
        result_image = detector.detect_and_highlight_edges(image_path)

        output_path = "/home/parth/Desktop/edge_detection/edge_detection/data/canny_edges_detected.jpg"
        detector.save_image(result_image, output_path)
        detector.show_image(result_image)

    except FileNotFoundError as e:
        print(e)
        sys.exit(1)
    except ValueError as e:
        print(e)
        sys.exit(1)
