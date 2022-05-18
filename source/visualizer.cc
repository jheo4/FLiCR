#include <visualizer.h>

using namespace flicr;

Visualizer::Visualizer()
{
  viewer = nullptr;
}


Visualizer::~Visualizer()
{

}


void Visualizer::initViewerXyz()
{
  viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PointCloudVisualizer"));
  viewer->setBackgroundColor(0, 0, 0);
}


void Visualizer::setViewer(types::PclPcXyz pc)
{
  viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointXYZ>(pc);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 60, 20,  0, 0, 0,  0, 0, 1);
}


void Visualizer::setViewer(types::PclPcXyzi pc)
{
  viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointXYZI>(pc);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 60, 20,  0, 0, 0,  0, 0, 1);
}


void Visualizer::setViewer(types::PclMesh mesh)
{
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  //viewer->addPolygonMesh(mesh, "mesh", 0);
  viewer->addPolylineFromPolygonMesh(*mesh, "mesh", 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 60, 20,  0, 0, 0,  0, 0, 1);
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


void Visualizer::saveToFile(std::string fileName)
{
  if(viewer != nullptr)
  {
    viewer->saveScreenshot(fileName);
  }
  else
  {
    debug_print("Viewer is not set...");
  }
}
