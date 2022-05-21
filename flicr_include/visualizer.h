#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <defs.h>
#include <types.h>

namespace flicr
{
class Visualizer
{
  protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
  public:
    Visualizer();
    ~Visualizer();
    void initViewerXyz();
    void setViewer(types::PclPcXyz pc);
    void setViewer(types::PclPcXyzi pc);
    void setViewer(types::PclMesh mesh);
    void show(int tickPeriod=100);
    void saveToFile(std::string fileName);
};
}

