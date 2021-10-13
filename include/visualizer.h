#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <defs.h>

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
    void setViewer(PclPcXYZ pc);
    void setViewer(PclPcXYZI pc);
    void setViewer(pcl::PolygonMesh mesh);
    void show(int tickPeriod=100);
    void saveToFile(std::string fileName);
};

#endif

