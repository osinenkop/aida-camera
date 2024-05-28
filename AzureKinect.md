# Microsoft Azure Kinect ROS2

<p align="center">
  <img src="https://cdn-dynmedia-1.microsoft.com/is/image/microsoftcorp/7824-ImmHero:VP4-1399x600" width="350" title="StereoLabs ZED">
</p>


* To download ***Microsoft Azure Kinect*** package: 
  * Git the branch
  ```sh
  cd ~/aida_ws/aida_camera
  git checkout origin/Azure_Kinect -- .
  ```

  * Install the resources:
  
  ```sh
  mkdir -p ~/aida_software/AzureKinect
  cd ~/aida_software/AzureKinect
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
  
  * For Body Tracking:
  ```sh
  wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb
  sudo dpkg -i libk4abt1.1_1.1.2_amd64.deb
  ```

  * To have a functional Depth Engine:
  ```sh 
  sudo ln -s /usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0 /usr/lib/x86_64-linux-gnu/libdepthengine.so.2.0
  ```

  ```sh
  git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
  cd Azure-Kinect-Sensor-SDK 
  sudo cp scripts/99-k4a.rules /etc/udev/rules.d/
  ```


  ```sh
  cd ~/aida_ws/aida_camera
  rosdep install --from-paths ./ --ignore-src -r -y
  ```


  <!-- * Download the source codes:
    ```zsh
    
    git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b humble
    rosdep install --from-paths ./ --ignore-src -r -y
    ``` -->

To make sure everything is installed correctly, type `k4aviewer` and connect your camera. It should be able to detect its serial number.


**NOTE**
If launching this package in a headless mode, you should also:
```sh
export DISPLAY=:<1 or 0>
```


<!-- **NOTE 2**

Посмотреть размер буфера:
```sh
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

Чтобы увеличить надо в файл `/etc/default/grub` в конец строки  
```sh
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
дописать usbcore.usbfs_memory_mb=1000
```

Now reattach the camera! -->