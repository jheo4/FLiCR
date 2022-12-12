#include <flicr>
#include <pcl/point_cloud.h>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("a, ref", "Reference PC file path", cxxopts::value<std::string>())
    ("b, comp", "Comparing PC file path", cxxopts::value<std::string>())
    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string ref  = parsedArgs["ref"].as<std::string>();
  std::string comp  = parsedArgs["comp"].as<std::string>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tReference PC file: " << ref << endl;
    cout << "\tComparing PC file: " << comp << endl;
  }


  PcReader pcReader;
  types::PclPcXyz refPc = NULL;
  types::PclPcXyz compPc = NULL;

  refPc = pcReader.readXyzFromXyziBin(ref);
  if(refPc == NULL)
  {
    if(debug) debug_print("reading input file (%s) failed..", ref.c_str());
    exit(1);
  }

  compPc = pcReader.readXyzFromXyziBin(comp);
  if(compPc == NULL)
  {
    if(debug) debug_print("reading input file (%s) failed..", comp.c_str());
    exit(1);
  }

  int refPcSize  = refPc->size();
  int compPcSize = compPc->size();
  float diff = (refPcSize > compPcSize) ? refPcSize-compPcSize : compPcSize-refPcSize;
  float diffRatio = (refPcSize > compPcSize) ? diff/refPcSize*100 : diff/compPcSize*100;

  printf("Pc Size Comp\tref(%d)\tcomp(%d)\tdiff(%d)\t%.2f\n", refPcSize, compPcSize, (int)diff, diffRatio);



  return 0;
}
