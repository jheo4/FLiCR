# FLiCR: A Fast and Lightweight LiDAR Point Cloud Compression Based on Lossy RI

## Installation
This project is on Linux and tested with ubuntu 18.04 and 20.04. You can get a container of the environmental setup
from [here](https://hub.docker.com/repository/docker/jin993/er_pcc) with the account "newuser" and password "newuser".
You have to connect X-window to this container. On Linux host, you can refer the following command.
```
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ --name my_container jin993/er_pcc:CONTAINER_IMG
```

If the above container does not work, install following dependencies by your environment.
- OpenCV4
- RosBag with Ros base https://wiki.ros.org/ROS/Installation https://wiki.ros.org/rosbag
  - When there is an issue about lz4, try followings.
    ```
    sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
    sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

    sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
    sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
    ```
- FFmpeg https://ffmpeg.org/download.html (tested with 4.3)
- PCL https://pointclouds.org/downloads/
    ```
    sudo apt install libpcl-dev libpcl-conversions-dev
    ```
- yamlcpp https://github.com/jbeder/yaml-cpp
    ```
    git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
    git checkout yaml-cpp-0.6.3
    mkdir build && cd build
    cmake .. -DYAML_BUILD_SHARED_LIBS=OFF && make -j$(nproc) && ./test/yaml-cpp-tests
    sudo make install
    ```
- spdlog
    ```
    git clone https://github.com/gabime/spdlog.git && cd spdlog
    git checkout v1.8.2
    mkdir build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    ```

Then, get the repo with the example data.
To run the example, you should set PCC_HOME as your PROJECT_DIR in ~/.bashrc
```
export PCC_HOME=YOUR_PROJ_DIR_LOCATION
```

### Example Data
Place downloaded sample data and set config.yaml.

#### KITTI
https://ericsson.sharepoint.com/sites/Atlantis/Shared%20Documents/XR%20E2E%20Solutions/Point%20Cloud%20Compression/kitti_2011_09_26_drive_0002_synced.bag

#### Set config.yaml for downloaded sample data
```
kitti_bag_file: "YOUR_PATH/kitti_2011_09_26_drive_0002_synced.bag"
```

If your data files have different names, set it with yours.


## To Build
```
cd PROJECT_DIR
mkdir builds && cd builds
cmake .. && make -j $(nproc)
# examples will be under build/example
./example/EXAMPLE_TO_RUN
```

