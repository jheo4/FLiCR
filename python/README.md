# Python scripts for 3D PCC

## KITTI visualization
Along with the installation of python libs, it would be required to install system-wide libs.
```
# Tested in Ubuntu 18.04 with Python 3.6
sudo apt install python3-pyqt5 vtk7

virtualenv ./venv --python=python3
. ./venv/bin/activate
pip install numpy pykitti opencv-python pillow scipy matplotlib jedi # or you can install pip -r requirements.txt

# Place your data to data directory. You can find the kitti dataset http://www.cvlibs.net/datasets/kitti/raw_data.php
# it should look like below.
# data
# `-- 2011_09_26
#   |-- 2011_09_26_drive_0002_sync
#   |   |-- image_00
#   |   |-- image_01
#   |   |-- image_02
#   |   |-- image_03
#   |   |-- oxts
#   |   |-- tracklet_labels.xml
#   |   `-- velodyne_points
#   |-- 2011_09_26_drive_0005_sync
#   |   |-- image_00
#   |   |-- image_01
#   |   |-- image_02
#   |   |-- image_03
#   |   |-- oxts
#   |   |-- tracklet_labels.xml
#   |   `-- velodyne_points
#   |-- calib_cam_to_cam.txt
#   |-- calib_imu_to_velo.txt
#   `-- calib_velo_to_cam.txt

python main.py
```

Reference: https://github.com/navoshta/KITTI-Dataset
