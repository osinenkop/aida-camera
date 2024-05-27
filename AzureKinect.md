# Microsoft Azure Kinect ROS2





* To download ***Microsoft Azure Kinect*** package: 
  * Install the resources:
  
  ```sh
  mkdir -p ~/aida_software/AzureKinekt
  cd ~/aida_software/AzureKinekt
  ```

  ```sh
  sudo apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main'
  sudo apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
  sudo apt --fix-broken install
  sudo apt-get install -y libsoundio1
  sudo apt-add-repository -r -y -n 'deb http://archive.ubuntu.com/ubuntu focal universe'
  sudo apt-add-repository -r -y 'deb http://archive.ubuntu.com/ubuntu focal main'
  ```

  ```sh
  wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb
  sudo dpkg -i libk4a1.4_1.4.1_amd64.deb
  ```

  ```sh
  wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb
  sudo dpkg -i libk4a1.4-dev_1.4.1_amd64.deb
  ```

  ```sh
  wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb
  sudo dpkg -i k4a-tools_1.4.1_amd64.deb
  ```

  ```sh
  git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
  cd Azure-Kinect-Sensor-SDK 
  sudo cp scripts/99-k4a.rules /etc/udev/rules.d/
  ```

  To make sure everything is installed correctly, type `k4aviewer` and connect your camera. It should be able to detect its serial number.

* Go back to `aida_ws`:
  ```sh
  cd ~/aida_ws
  ```
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
