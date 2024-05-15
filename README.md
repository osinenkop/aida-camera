# aida-camera
A go-to repository for ready-to-use ROS2 implementations of various cameras

First of all, if you ***don't have OpenCV*** installed, follow the next step. 
## Install OpenCV
Follow the instructions in [Here](OpenCV.md)

## Prepare the WorkSpace
To be able to use the cameras in ROS2, you will need to:
```zsh
mkdir ~/aida_ws && cd ~/aida_ws
```

```zsh
git clone https://github.com/osinenkop/aida-camera.git -b main
```

## Clone Camera packages
In order to download each desired camera package:
```zsh
cd ~/aida_ws/aida-camera
```

and to see the list of available cameras:

```zsh
git branch -a
```

Then:

* To download ***Jabra*** package: 
  * Git the branch:
    ```sh
    git checkout origin/Jabra -- .
    ```

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
* To download ***GigE*** package: 
  * Git the branch:
    ```sh
    git checkout origin/GigE -- .
    ```

  * Install the resources:
    ```sh
    mkdir -p ~/aida_software/MVSDK
    tar -xf linuxSDK_V2.1.0.41.tar.gz -C ~/aida_software/MVSDK/
    cd ~/aida_software/MVSDK/
    ```
  * Follow the instructions in `LinuxSDK Document.pdf`
    

## COPY and MERGE the `launch` and `config` 
Now ***COPY and MERGE the `launch` and `config` folders*** to your own package. You may need to add more configuration files to the `config` directory.

***To be able to use these packages***, you will also need to have  the `launch` and `config` in your package CMakeLists.txt. If you haven't done so, should have the following:

```cmake
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
```

## Build
```sh
cd ~/aida_ws && colcon build --symlink-install
echo source $PWD/install/setup.sh >> ~/.<your_shell>rc
```



