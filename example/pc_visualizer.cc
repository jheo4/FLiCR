#include "types.h"
#include <flicr>
#include <pcl/point_cloud.h>

using namespace std;
using namespace flicr;


void visualizeXyz(types::PclPcXyz pcXyz, bool colorized, int ms)
{
  Visualizer visualizer;
  visualizer.initViewerXyz();

  if(colorized)
  {
    types::PclPcXyzRgb point_cloud_ptr (new pcl::PointCloud<types::PclXyzRgb>);

    for(int i = 0; i < pcXyz->size(); i++)
    {
      pcl::PointXYZRGB point;

      point.x   = pcXyz->points[i].x;
      point.y   = pcXyz->points[i].y;
      point.z   = pcXyz->points[i].z;
      float rho = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
      rho = rho/80;
      uint8_t nRho = rho * 255;

      std::uint32_t rgb = (static_cast<std::uint32_t>(nRho) << 16 | static_cast<std::uint32_t>(255-nRho) << 8 | static_cast<std::uint32_t>(200));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    visualizer.setColorizedViewer(point_cloud_ptr);
  }
  else
  {
    visualizer.setViewer(pcXyz);
  }
  visualizer.show(ms);
}


void visualizeXyzi(types::PclPcXyzi pcXyzi, bool colorized, int ms)
{
  if(colorized)
  {
    Visualizer visualizer;
    visualizer.initViewerXyz();
    types::PclPcXyzRgb point_cloud_ptr (new pcl::PointCloud<types::PclXyzRgb>);

    for(int i = 0; i < pcXyzi->size(); i++)
    {
      pcl::PointXYZRGB point;

      point.x   = pcXyzi->points[i].x;
      point.y   = pcXyzi->points[i].y;
      point.z   = pcXyzi->points[i].z;
      uint8_t intensity = 300 * pcXyzi->points[i].intensity;

      std::uint32_t rgb = (static_cast<std::uint32_t>(intensity) << 16 | static_cast<std::uint32_t>(255-intensity) << 8 | static_cast<std::uint32_t>(200));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    visualizer.setColorizedViewer(point_cloud_ptr);
    visualizer.show(ms);
  }
  else
  {
    types::PclPcXyz pcXyz = types::xyzi2xyz(pcXyzi);
    visualizeXyz(pcXyz, colorized, ms);
  }

}


int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("s, millisecond", "Time to display in millisecond", cxxopts::value<int>())
    ("c, colorize", "Colorize the point cloud", cxxopts::value<bool>()->default_value("false"))
    ("r, xyz", "Is the input XYZ?", cxxopts::value<bool>()->default_value("false"))
    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string input  = parsedArgs["input"].as<std::string>();
  int ms             = parsedArgs["millisecond"].as<int>();
  bool colorized     = parsedArgs["colorize"].as<bool>();
  bool isXyz         = parsedArgs["xyz"].as<bool>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput file: " << input << endl;
    cout << "\ttime to display in seciond: " << ms << endl;
  }


  PcReader pcReader;

  if(isXyz == true)
  {
    types::PclPcXyz xyz = NULL;
    xyz = pcReader.readXyzBin(input);
    if(xyz == NULL)
    {
      if(debug) debug_print("reading input file (%s) failed..", input.c_str());
      exit(1);
    }

    visualizeXyz(xyz, colorized, ms);
  }
  else
  {
    types::PclPcXyzi xyzi = NULL;
    xyzi = pcReader.readXyziBin(input);
    if(xyzi == NULL)
    {
      if(debug) debug_print("reading input file (%s) failed..", input.c_str());
      exit(1);
    }

    visualizeXyzi(xyzi, colorized, ms);
  }

  return 0;
}

