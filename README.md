# 3D_PCC

## Installation
### Dependencies (Tested with Ubuntu 18.04)
1. OpenCV
```
# Install depending libs
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev

# Get OpenCV
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/opencv_build/opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..
make && sudo make install
```

2. OpenGL
```
sudo apt install libglfw3-dev libgles2-mesa-dev mesa-utils freeglut3-dev \
    libglu1-mesa libglu1-mesa-dev mesa-common-dev mesa-utils-extra libglew1.5 \
    libglew1.5-dev libgl1-mesa-glx libgl1-mesa-dev glmark2 glmark2-es2
```

3. RosBag
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-ros-base ros-melodic-rosbag
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

5. Intel RealSense SDK
```
# Approach 1: Package repository
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Approach 2: From Source
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
make && sudo make install
```

6. FFMpeg

### Example Data
Place downloaded sample data into the directory "resources".
#### D435i
```
wget https://librealsense.intel.com/rs-tests/TestData/d435i_sample_data.zip
```

#### KITTI
https://ericsson.sharepoint.com/sites/Atlantis/Shared%20Documents/XR%20E2E%20Solutions/Point%20Cloud%20Compression/kitti_2011_09_26_drive_0002_synced.bag
