#include <flicr>
#include <pcl/point_cloud.h>

using namespace std;
using namespace flicr;

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
  types::PclPcXyz xyz = NULL;

  if(isXyz == true)
    xyz = pcReader.readXyzBin(input);
  else
    xyz = pcReader.readXyzFromXyziBin(input);

  if(xyz == NULL)
  {
    if(debug) debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }

  Visualizer visualizer;
  visualizer.initViewerXyz();
  if(colorized)
  {
    types::PclPcXyzRgb point_cloud_ptr (new pcl::PointCloud<types::PclXyzRgb>);

    for(int i = 0; i < xyz->size(); i++)
    {
      pcl::PointXYZRGB point;

      point.x   = xyz->points[i].x;
      point.y   = xyz->points[i].y;
      point.z   = xyz->points[i].z;
      float rho = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
      rho = rho/80;
      uint8_t nRho = rho * 255;
      // uint32_t r, g, b;
      // r = xyz->points[i].x;
      // g = xyz->points[i].y;
      // b = xyz->points[i].z;
      // std::uint8_t r(255), g(15), b(15);
      // std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
      std::uint32_t rgb = (static_cast<std::uint32_t>(nRho) << 16 | static_cast<std::uint32_t>(255-nRho) << 8 | static_cast<std::uint32_t>(150+nRho));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    visualizer.setColorizedViewer(point_cloud_ptr);
  }
  else
  {
    visualizer.setViewer(xyz);
  }
  visualizer.show(ms);

  return 0;
}
