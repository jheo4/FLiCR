#include <flicr>
#include <pcl/point_cloud.h>

using namespace std;
using namespace flicr;


int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("s, millisecond", "Time to display in millisecond", cxxopts::value<int>())
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

  pcl::PolygonMeshPtr mesh = nullptr;
  std::vector<int> partID, pointStates;
  pcReader.generateMeshFromXyz(xyz, mesh, partID, pointStates);

  Visualizer visualizer;
  visualizer.initViewerXyz();
  visualizer.setViewer(mesh);
  visualizer.show(ms);

  return 0;
}

