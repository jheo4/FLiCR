#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("decompressor", "FLiCR Decompressor (XYZ)");
  options.add_options()
    ("i, input", "Encoded input file path", cxxopts::value<std::string>())
    ("o, output", "Decoded output file path", cxxopts::value<std::string>())

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
    cout << "\tsensor's yaw FoV: "   << yaw_fov << endl;
    cout << "\tsensor's pitch FoV: " << pitch_fov << endl;
    cout << "\tsensor's yaw offset: "   << yaw_offset << endl;
    cout << "\tsensor's pitch offset: " << pitch_offset << endl;
    cout << "\tRI resolution: " << x << ", " << y << endl;
    cout << "\tCompression: " << compression << endl;
  }

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./decompressor_xyz.log");

  double st, et;

  float yaw = yaw_fov / x;
  float pitch = pitch_fov / y;

  PcWriter pcWriter;
  types::PclPcXyz xyz = NULL;

  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  cv::Mat ri;
  double conv, comp;

  FILE* ifp = fopen(input.c_str(), "rb");
  if(ifp == NULL)
  {
    if(debug) debug_print("opening input file failed..");
    exit(1);
  }
  int readBufSize = 4800000;
  char *readBuf = new char[readBufSize];
  int encodedSize;
  encodedSize = fread(readBuf, sizeof(char), readBufSize, ifp);

  if(compression == std::string("lz77") || compression == std::string("LZ77"))
  {
    BoostZip boostZip;
    std::vector<char> encoded(readBuf, readBuf + encodedSize);
    std::vector<char> decoded;

    st = getTsNow();
    boostZip.inflateGzip(encoded, decoded);
    et = getTsNow();
    comp = et-st;

    st = getTsNow();
    cv::Mat nRi(y, x, CV_8UC1, decoded.data());
    riConverter.denormalizeRi(nRi, min, max, ri);
    xyz = riConverter.reconstructPcFromRi(ri, true);
    et = getTsNow();
    conv = et-st;

    pcWriter.writeBin(output, xyz);
  }
  else if(compression == std::string("rle_ri") || compression == std::string("RLE_RI"))
  {
    RunLengthCompressor rleCompressor;
    std::vector<char> encoded(readBuf, readBuf + encodedSize);

    st = getTsNow();
    std::vector<char> decoded = rleCompressor.decode(encoded);
    et = getTsNow();
    comp = et-st;

    st = getTsNow();
    cv::Mat nRi(y, x, CV_8UC1, decoded.data());
    riConverter.denormalizeRi(nRi, min, max, ri);
    xyz = riConverter.reconstructPcFromRi(ri, true);
    et = getTsNow();
    conv = et-st;

    pcWriter.writeBin(output, xyz);
  }

  latencyLogger->info("{}\t{}\t{}\t{}", compression, comp, conv, conv+comp);
  if(debug) debug_print("comp %f, conv %f, e2e %f", comp, conv, conv+comp);

  delete[] readBuf;
  fclose(ifp);
  return 0;
}
