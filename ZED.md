# ZED ROS2
<p align="center">
  <img src="https://cdn.sanity.io/images/s18ewfw4/production/e6844357d5600805f041ff2ec9ad1745422fba82-4000x4000.png?rect=412,861,3578,1789&w=2880&h=1440&q=80&auto=format" width="350" title="StereoLabs ZED">
</p>

* To download ***ZED*** package: 
  * Git the branch
    ```sh
    git checkout origin/ZED -- .
    ```
  * Install CUDA
  * Install [ZED SDK](https://www.stereolabs.com/developers/release).
  * Download the source codes:
    ```zsh
    cd ~/aida_ws/aida_camera
    git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
    rosdep install --from-paths ./ --ignore-src -r -y
    ```
