#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef __PCC_VISUALIZER__
#define __PCC_VISUALIZER__

class Visualizer
{
  protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
  public:
    Visualizer();
    ~Visualizer();
    void initViewerXYZ();
    void setViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc);
    void show(int tickPeriod=100);
};

#endif

