#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("r, ref_pc", "Reference input file path", cxxopts::value<std::string>())
    ("c, comp_pc", "Comparing input file path", cxxopts::value<std::string>())
    ("m, max", "Max sensing distance", cxxopts::value<float>())
    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string refPcFile   = parsedArgs["ref_pc"].as<std::string>();
  std::string compPcFile  = parsedArgs["comp_pc"].as<std::string>();
  float max = parsedArgs["max"].as<float>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tReference file: " << refPcFile << endl;
    cout << "\tComparing file: " << compPcFile << endl;
  }

  PcReader pcReader;
  types::PclPcXyz refPc = NULL;
  types::PclPcXyz compPc = NULL;

  refPc = pcReader.readXyzFromXyziBin(refPcFile);
  compPc = pcReader.readXyzFromXyziBin(compPcFile);
  if(refPc == NULL || compPc == NULL)
  {
    if(debug) debug_print("reading input file (%s or %s) failed..", refPcFile.c_str(), compPcFile.c_str());
    exit(1);
  }

  float cd = flicr::Metrics::calcCdBtwPcs(refPc, compPc);
  float psnr = flicr::Metrics::calcPsnrBtwPcs(refPc, compPc, max);

  cout << "Metric Results" << endl;
  cout << "\t PSNR " << psnr << ", CD " << cd << endl;
  cout << "\t refPointSize " << refPc->size() << ", compPointSize " << compPc->size() << ", diffAbs " << abs(int(refPc->size() - compPc->size())) << endl;

  return 0;
}
