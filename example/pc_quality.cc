#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("r, ref_pc", "Reference input file path", cxxopts::value<std::string>())
    ("r_xyz", "Is reference PC only XYZ?", cxxopts::value<bool>()->default_value("false"))
    ("c, comp_pc", "Comparing input file path", cxxopts::value<std::string>())
    ("c_xyz", "Is comparing PC only XYZ?", cxxopts::value<bool>()->default_value("false"))
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
  bool rXyz = parsedArgs["r_xyz"].as<bool>();
  bool cXyz = parsedArgs["c_xyz"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tReference file: " << refPcFile << endl;
    cout << "\tComparing file: " << compPcFile << endl;
  }

  PcReader pcReader;
  types::PclPcXyz refPc = NULL;
  types::PclPcXyz compPc = NULL;

  if(rXyz == true)
    refPc = pcReader.readXyzBin(refPcFile);
  else
    refPc = pcReader.readXyzFromXyziBin(refPcFile);

  if(cXyz == true)
    compPc = pcReader.readXyzBin(compPcFile);
  else
    compPc = pcReader.readXyzFromXyziBin(compPcFile);

  if(refPc == NULL || compPc == NULL)
  {
    if(debug) debug_print("reading input file (%s or %s) failed..", refPcFile.c_str(), compPcFile.c_str());
    exit(1);
  }

  float cd = flicr::Metrics::calcCdBtwPcs(refPc, compPc);
  float psnr = flicr::Metrics::calcPsnrBtwPcs(refPc, compPc, max);

  printf("PSNR\t%f\tCD\t%f\n", psnr, cd);
  return 0;
}
