# RealSense ROS2
* To download ***RealSense*** package: 
  * Git the branch
    ```sh
    git checkout origin/RealSense -- .
    ```
  * Download the source codes:
    ```zsh
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
    ```
  * Install the resources:
    ```sh
    sudo apt install ros-$ROS_DISTRO-librealsense2 ros-$ROS_DISTRO-librealsense2-dbgsym ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-camera-dbgsym ros-$ROS_DISTRO-realsense2-camera-msgs ros-$ROS_DISTRO-realsense2-camera-msgs-dbgsym ros-$ROS_DISTRO-realsense2-description
    ```