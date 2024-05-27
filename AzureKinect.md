# Microsoft Azure Kinect ROS2





* To download ***Microsoft Azure Kinect*** package: 
  * Git the branch
    ```sh
    git checkout origin/Azure_Kinect -- .
    ```
  * Download the source codes:
    ```zsh
    cd ~/aida_ws/aida_camera
    git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b humble
    rosdep install --from-paths ./ --ignore-src -r -y
    ```
  * Install the resources:
    ```sh
    sudo apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main'
    sudo apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
    sudo apt --fix-broken install
    sudo apt-get install -y libsoundio1 libk4a1.3 libk4a1.3-dev k4a-tools
    sudo apt-add-repository -r -y -n 'deb http://archive.ubuntu.com/ubuntu focal universe'
    sudo apt-add-repository -r -y 'deb http://archive.ubuntu.com/ubuntu focal main'
    ```
