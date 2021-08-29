# 3D_PCC

## Installation
This project is on Linux and tested with ubuntu 18.04 and 20.04. You can get a container of the environmental setup
from [here](https://hub.docker.com/repository/docker/jin993/er_pcc) with the account "newuser" and password "newuser".
You have to connect X-window to this container. On Linux host, you can refer the following command.
```
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ --name my_container jin993/er_pcc:CONTAINER_IMG
```

If the above container does not work, install following dependencies by your environment.
- OpenCV4
- OpenGL
- RosBag with Ros base https://wiki.ros.org/ROS/Installation https://wiki.ros.org/rosbag
- Intel RealSense SDK https://github.com/IntelRealSense/librealsense
- FFmpeg https://ffmpeg.org/download.html (tested with 4.3)
- PCL https://pointclouds.org/downloads/
- yamlcpp https://github.com/jbeder/yaml-cpp

Then, get the repo with the example data.
To run the example, you should set PCC_HOME as your PROJECT_DIR in ~/.bashrc
```
export PCC_HOME=YOUR_PROJ_DIR_LOCATION
```

### Example Data
Place downloaded sample data and set config.yaml.
#### D435i
```
wget https://librealsense.intel.com/rs-tests/TestData/d435i_sample_data.zip
```

#### KITTI
https://ericsson.sharepoint.com/sites/Atlantis/Shared%20Documents/XR%20E2E%20Solutions/Point%20Cloud%20Compression/kitti_2011_09_26_drive_0002_synced.bag

#### Set config.yaml for downloaded sample data
```
kitti_bag_file: "YOUR_PATH/kitti_2011_09_26_drive_0002_synced.bag"
l515_bag_file: "YOUR_PATH/l515-test.bag"
d435i_bag_file: "YOUR_PATH/d435i_walking.bag"
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

