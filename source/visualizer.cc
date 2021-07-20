#include <visualizer.h>
#include <utils.h>

Visualizer::Visualizer()
{
  viewer = nullptr;
}


Visualizer::~Visualizer()
{

}


void Visualizer::initViewerXYZ()
{
  viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PointCloudVisualizer"));
  viewer->setBackgroundColor(0, 0, 0);
}

void Visualizer::setViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc)
{
  viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointXYZ>(pc);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 40, 3,  0, 0, 0,  0, 0, 1);
}


void Visualizer::show(int tickPeriod)
{
  if(viewer != nullptr)
  {
    viewer->spinOnce(tickPeriod);
    std::this_thread::sleep_for(std::chrono::milliseconds(tickPeriod));
  }
  else
  {
    debug_print("Viewer is not set...");
  }
}
