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
    std::string fn;
    int imgIdx;

    Visualizer();
    ~Visualizer();
    void initViewerXyz();
    void setViewer(types::PclPcXyz pc);
    void setViewer(types::PclPcXyzi pc);
    void setViewer(types::PclMesh mesh);
    void setViewerBEV(int height=100);
    void setViewerXyz(int x, int y, int z);
    void show(int tickPeriod);
    void saveToFile(std::string fileName);

    void static keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* curVisualizer)
    {
      Visualizer *visualizer = (Visualizer*)(curVisualizer);
      if (event.getKeySym () == "s" && event.keyDown ())
      {
        std::string fileName = visualizer->fn + "_" + std::to_string(visualizer->imgIdx) + ".png";
        std::cout << "SAVE Screenshot as " << fileName << std::endl;
        visualizer->viewer->saveScreenshot(fileName);
        visualizer->imgIdx++;
      }
    }
};
}

