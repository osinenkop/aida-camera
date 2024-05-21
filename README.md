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

It's time to download the packages:
* [Jabra Package](Jabra.md)
* [RealSense Package](RealSense.md)
* [GigE Package](GigE.md)

    

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
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo source $PWD/install/setup.sh >> ~/.<your_shell>rc
```



