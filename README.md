# Vision Programming Challenge
 
## Introduction
This project provides a ROS-based pipeline for edge detection in images, 3D reconstruction using depth images, and visualization in RViz. The system utilizes Canny edge detection and depth-camera data to align the extracted 3D edges with the robot enviornment.

## Preliminary Requirements
1. ROS Noetic (Ubuntu 20.04)
2. Install python3
3. Create a catkin_workspace 
4. Clone this repository into the src/directory of your workspace
5. Download the bag files and other robot files.
6. Install all the python libraries from requierments.txt

## Setting up the enviornment
I have ROS2 Humble and Ubuntu 22.04 installed on my computer. To operate the environment, I had to utilize a Docker ROS Noetic image. The steps to create a docker container for ROS are listed below. You can omit this step if you already have ROS Noetic installed.

1. Create a docker-compose.yml file (The name of the ROS Image from which the container is run, the name of the container, prerequisites for running the code are all written inside the docker compose file for setting up the container as per your need)
2. To run your docker container
    ```
    Terminal 1
    # Start the container defined in the docker-compose.yml file
    sudo docker compose -f docker-compose.yml up 

    Terminal 2
    # Open an interactive Bash shell inside the running 'my-ros-container' (this is the name of my container) Docker container
    sudo docker exec -it my-ros-container bash 
    ```

## Canny Edge Detecion

Inside **catkin_workspace/src/edge_detecion/edge_detection/src/** you will find a **edge_detector.py** file which is used for edge detection. The code is optimized to detect the edges of a checkboard and highlight them with a green line superimposed on the image. 


### Features:
- Converts images to grayscale
- Applies **Gaussian Blur** to reduce noise
- Uses **Adaptive Canny Edge Detection**
- Superimposes detected edges onto the original image
- When the **edge_detector.py** file is run, we pass the path to the image as a **command-line argument** for flexibilty
- Saves and optionally displays the processed image

### How it Works:
1. **Preprocessing**: Converts the image to grayscale.
2. **Gaussian BLur**: Reduce noise and smoothens the image.
3. **Canny Edge Detection**:
     - Computes intensity gradients.
     - Applies non-maximum supression.
     - Uses adaptive thresholding for robust edge detection.
     - Performs edge tracking by hysteresis.
4. **Overlay Edges on the original Image**.
5. **Save or Display the Result**

### Example

## ROS Edge Detecion Service and Client
This provides a **ROS-based Edge Detection Service and Client** using the above mentioned **Canny Edge Detection Algorithm**. The service can process:
 - **A single image** and return the edge-detected result.
 - **A directory of images** , applying edge detection to all supoorted image files.

### Features
- ROS-based **Edge Detection Service**
- Accepts **both image files and directories**
- Uses **Canny Edge Detection** for robust edge processing
- Saves **processed images** with '_edges' suffix
- Handles errors and invalid inputs gracefully

### Installaton & Setup

1. Install Dependencies
    ```
    sudo apt update
    sudo apt install ros-noetic-cv-bridge ros-noetic-sensor-msgs
    pip install opencv-python numpy
    ```
2. Clone and Build the Package
    ```
    cd ~/catkin_workspace/src
    git clone https://github.com/Pvpatel123/edge_detection.git
    cd ~/catkin_workspace
    catkin_make
    source devel/setup.bash
    ```
3. Configure CMakeLists.txt
    ```
    find_package(catkin REQUIRED COMPONENTS
      rospy
      std_msgs
      sensor_msgs
      message_generation
      pcl_conversions #below this will be used for further tasks
      pcl_ros
      cv_bridge
    )

    add_message_files(
        Files
        EdgeDetectionResult.msg
    )

    add_service_files(
      FILES
      EdgeDetection.srv
    )

    generate_messages(
      DEPENDENCIES
      std_msgs
      sensor_msgs
    )

    catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge pcl_ros pcl_conversions message_runtime)
    ```
4. Modify package.xml:
    ```
    <buildtool_depend>catkin</buildtool_depend>
    <depend>roscpp</depend> 
    <depend>message_generation</depend> 
    <depend>message_runtime</depend> 
    <depend>rospy</depend> 
    <depend>std_msgs</depend> 
    <depend>sensor_msgs</depend> 
    <depend>cv_bridge</depend> 
    <depend>image_transport</depend>
    <depend>pcl_ros</depend>
    <depend>pcl_conversions</depend>
    ```
### Running the ROS Edge Detection Service

1. Terminal 1
    ```
    roscore

    ```
2. Terminal 2
    ```
    rosrun edge_detection edge_detection_server.py

    ```
3. Terminal 3
    ```
    rosrun edge_detection edge_detection_client.py /path/to/directory or image

    ```
### Examples



## ROS Edge Detection with 3D Projection and RViz Visualization

This extends ** Canny Edge Detection by:
- **Detecting edges from image topics** and publishing them.
- **Converting edge pixels to 3D coordinates** using depth images and camera intrinsics.
- **Publishing 3D edge points as 'sensor_msg/PointCloud'**.
- **Visualizing images, 3D points and the robot model in RViz**.

### Features
- Subcirbes to image topics for real-time edge detection.
- Applies **Canny edge detection** and publishes the results.
- Uses depth images and camera intrinsics to compute 3D coordinates (camera Intinsics are read from **'camera/color/camera_info'**)
- Publishes 3D edge points to **'/edge_points' (PointCloud)**
- Visualizes detected edges in RViz
- Load a robot URDF model in RViz

### Running The Full Edge Detection with 3D Projection aand Robot Visualization

1. Do the setup same as the ROS Edge Detection Service and Client
2. Start ROS Master
   ```
   roscore
   ```
3. Play the ROS Bag
   ```
   rosbag play --clock -l <path to bagfile>
   ```
   
4. Run the Edge Detection Node
   ```
   rosrun edge_detection edge_detection_node.py
   ```
5. Run the Edge-to-3D Conversion Node
   ```
   rosrun edge_detection edge_to_3d_node.py
   ```
6. Launch Rviz with Robot Model and edge markers
   ```
   roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false
   ```

### Example


## Possible Improvements
1. We can integrate Sobel filter and Gaussian blur before applying Canny Edge Detection for simple Edge detection geometry as Sobel give more accurate detection but it is sensitive to noise .
2. There is a small delay between the 3D edge points and the camera image because of the slow processing of the PC. It can be imrovped by upgrading the processing capacity of the system.
