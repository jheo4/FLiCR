#ifndef __PCC_DEFS__
#define __PCC_DEFS__

#define PI 3.14159266

#define KITTI_DATASET_FREQUENCY 10

// https://bit.ly/3gHtGzf
#define HDL64_THETA_PRECISION 0.4187

#define HDL64_PI_PRECISION 0.08

#define HDL64_PI_PRECISION_4500 0.08
#define HDL64_PI_PRECISION_4096 0.087890625
#define HDL64_PI_PRECISION_2048 0.17578125
#define HDL64_PI_PRECISION_1024 0.3515625
#define HDL64_PI_PRECISION_512  0.703125
#define HDL64_PI_PRECISION_256  1.40625

#define HDL64_VERTICAL_DEGREE_OFFSET (-88.0f)
#define HDL64_HORIZONTAL_DEGREE_OFFSET 180.0f
#define HDL64_VERTICAL_DEGREE   26.8f
#define HDL64_HORIZONTAL_DEGREE 360.0f

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
using PclXYZ    = pcl::PointXYZ;
using PclXYZI   = pcl::PointXYZI;
using PclPcXYZ  = pcl::PointCloud<PclXYZ>::Ptr;
using PclPcXYZI = pcl::PointCloud<PclXYZI>::Ptr;
using PclMesh   = pcl::PolygonMeshPtr;
#endif

