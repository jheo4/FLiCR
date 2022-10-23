#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("o, output", "Encoded output file path", cxxopts::value<std::string>())
    ("yaw", "yaw precision", cxxopts::value<float>())
    ("pitch", "pitch precision", cxxopts::value<float>())
    ("yaw_fov",   "yaw fov", cxxopts::value<float>())
    ("pitch_fov", "pitch fov", cxxopts::value<float>())
    ("yaw_offset",   "yaw offset", cxxopts::value<float>())
    ("pitch_offset", "pitch offset", cxxopts::value<float>())
    ("min_range", "minimum range", cxxopts::value<float>())
    ("max_range", "maximum range", cxxopts::value<float>())

    ("x, width", "range image width", cxxopts::value<int>())
    ("y, height", "range image height", cxxopts::value<int>())
    ("c, compression", "compression: lz77 | rle_ri", cxxopts::value<std::string>())
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
  std::string output = parsedArgs["output"].as<std::string>();
  float yaw   = parsedArgs["yaw"].as<float>();
  float pitch = parsedArgs["pitch"].as<float>();
  float yaw_fov   = parsedArgs["yaw_fov"].as<float>();
  float pitch_fov = parsedArgs["pitch_fov"].as<float>();
  float yaw_offset   = parsedArgs["yaw_offset"].as<float>();
  float pitch_offset = parsedArgs["pitch_offset"].as<float>();
  float min   = parsedArgs["min_range"].as<float>();
  float max   = parsedArgs["max_range"].as<float>();
  int x = parsedArgs["width"].as<int>();
  int y = parsedArgs["height"].as<int>();
  std::string compression = parsedArgs["compression"].as<std::string>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput file: " << input << endl;
    cout << "\toutput file: " << output << endl;
    cout << "\tsensor's yaw precision: "   << yaw << endl;
    cout << "\tsensor's pitch precision: " << pitch << endl;
    cout << "\tsensor's yaw FoV: "   << yaw_fov << endl;
    cout << "\tsensor's pitch FoV: " << pitch_fov << endl;
    cout << "\tsensor's yaw offset: "   << yaw_offset << endl;
    cout << "\tsensor's pitch offset: " << pitch_offset << endl;
    cout << "\tRI resolution: " << x << ", " << y << endl;
    cout << "\tCompression: " << compression << endl;
  }

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./compressor_xyzi2xyz.log");

  double st, et;

  PcReader pcReader;
  types::PclPcXyz xyz = NULL;

  xyz = pcReader.readXyzFromXyziBin(input);
  if(xyz == NULL)
  {
    if(debug) debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }
  FILE* ofp = fopen(output.c_str(), "wb");
  if(ofp == NULL)
  {
    if(debug) debug_print("output file creation failed..");
    exit(1);
  }

  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  cv::Mat ri, nRi;
  double riMin, riMax;
  double conv, comp;

  if(compression == std::string("lz77") || compression == std::string("LZ77"))
  {

    BoostZip boostZip;
    std::vector<char> dictRes;
    st = getTsNow();
    riConverter.convertPc2Ri(xyz, ri, true);
    riConverter.normalizeRi(ri, nRi, riMin, riMax);
    et = getTsNow();
    conv = et-st;
    st = getTsNow();
    boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), dictRes);
    et = getTsNow();
    comp = et-st;

    fwrite(dictRes.data(), sizeof(char), dictRes.size(), ofp);

    latencyLogger->info("{}\t{}\t{}\t{}\t{}", compression, conv, comp, conv+comp, dictRes.size());
    if(debug) debug_print("conv %f, comp %f, e2e %f", conv, comp, conv+comp);
  }
  if(compression == std::string("rle_ri") || compression == std::string("RLE_RI"))
  {
    RunLengthCompressor rleCompressor;
    st = getTsNow();
    riConverter.convertPc2Ri(xyz, ri, true);
    riConverter.normalizeRi(ri, nRi, riMin, riMax);
    et = getTsNow();
    conv = et-st;
    st = getTsNow();
    std::vector<char> encoded = rleCompressor.encode((char*)nRi.data, nRi.elemSize()*nRi.total());
    et = getTsNow();
    comp = et-st;
    fwrite(encoded.data(), sizeof(char), encoded.size(), ofp);

    latencyLogger->info("{}\t{}\t{}\t{}\t{}", compression, conv, comp, conv+comp, encoded.size());
    if(debug) debug_print("conv %f, comp %f, e2e %f, size %ld", conv, comp, conv+comp, encoded.size());
  }
  fclose(ofp);
  return 0;
}
