# aida-camera
A go-to repository for ready-to-use ROS2 implementations of various cameras

## Install OpenCV
If you ***don't have OpenCV*** installed, follow the instructions in [Here](OpenCV.md)

## Prepare the WorkSpace
Then, to be able to use the cameras in ROS2, you will need to:
```zsh
mkdir ~/aida_ws && cd ~/aida_ws
```

```zsh
git clone -b main https://github.com/osinenkop/aida-camera.git
```

## Clone Camera packages
In order to download each desired camera package(s):
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