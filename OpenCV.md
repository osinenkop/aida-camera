# Install OpenCV
```sh
mkdir -p ~/aida_software/OpenCV
cd ~/aida_software/OpenCV
```

```sh
wget https://github.com/opencv/opencv/archive/4.9.0.zip
```

```sh
unzip 4.9.0.zip
rm -rf 4.9.0.zip
```

```sh
cd opencv-4.9.0
git clone https://github.com/opencv/opencv_contrib.git
mkdir build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules  ../
cmake --build . -j4
```