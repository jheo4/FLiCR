#include "opencv2/imgcodecs.hpp"
#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("o, output", "Output file path", cxxopts::value<std::string>())
    ("yaw_fov",   "yaw fov", cxxopts::value<float>())
    ("pitch_fov", "pitch fov", cxxopts::value<float>())
    ("yaw_offset",   "yaw offset", cxxopts::value<float>())
    ("pitch_offset", "pitch offset", cxxopts::value<float>())
    ("min_range", "minimum range", cxxopts::value<float>())
    ("max_range", "maximum range", cxxopts::value<float>())

    ("x, width", "range image width", cxxopts::value<int>())
    ("y, height", "range image height", cxxopts::value<int>())

    ("intr_x", "interpolated width", cxxopts::value<int>())
    ("intr_y", "interpolated height", cxxopts::value<int>())
    ("a, algorithm", "CV algorithm to use [linear, cubic, lz4, area, nearest]", cxxopts::value<std::string>())

    ("n, normalize", "normalize ri/intmap", cxxopts::value<bool>())

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

  int intrX = parsedArgs["intr_x"].as<int>();
  int intrY = parsedArgs["intr_y"].as<int>();
  std::string algorithm = parsedArgs["algorithm"].as<std::string>();

  bool norm = parsedArgs["normalize"].as<bool>();
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
    cout << "\tsensor's min/max range: " << min << ", " << max << endl;
    cout << "\tRI, intMap resolution: " << x << ", " << y << endl;

    cout << "\tCV algorithm: " << algorithm  << ", intRes: " << intrX << ", " << intrY << endl;

    cout << "\tNormalize: " << norm << endl;
  }


  PcReader pcReader;
  types::PclPcXyzi xyzi = NULL;

  xyzi = pcReader.readXyziBin(input);
  if(xyzi == NULL)
  {
    if(debug) debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }

  float pitch = pitch_fov / y;
  float yaw   = yaw_fov / x;
  cout << pitch << ", " << yaw << endl;

  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  cv::Mat ri, intMap;
  cv::Mat outRi, outIntMap;
  cv::Mat intrRi, intrIntMap;
  double riMin, riMax;
  double intMapMin, intMapMax;

  riConverter.convertPc2RiWithIm(xyzi, ri, intMap, true);

  if(norm)
  {
    riConverter.normalizeRi(ri, outRi, riMin, riMax);
    riConverter.normalizeRi(intMap, outIntMap, intMapMin, intMapMax);
  }
  else
  {
    outRi = ri;
    outIntMap = intMap;
  }

  int intrAlg;
  // INTER_LINEAR – a bilinear interpolation (used by default)
  // INTER_AREA – resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
  // INTER_CUBIC – a bicubic interpolation over 4×4 pixel neighborhood
  // INTER_LANCZOS4 – a Lanczos interpolation over 8×8 pixel neighborhood
  // INTER_NEAREST – a nearest-neighbor interpolation
  if(algorithm == "linear" || algorithm == "LINEAR") intrAlg = cv::INTER_LINEAR;
  else if(algorithm == "cubic" || algorithm == "CUBIC") intrAlg = cv::INTER_CUBIC;
  else if(algorithm == "lz4" || algorithm == "LZ4") intrAlg = cv::INTER_LANCZOS4;
  else if(algorithm == "area" || algorithm == "AREA") intrAlg = cv::INTER_AREA;
  else if(algorithm == "nearest" || algorithm == "NEAREST") intrAlg = cv::INTER_NEAREST;
  else intrAlg = cv::INTER_LINEAR;

  // https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
  cv::resize(ri, intrRi, cv::Size(intrX, intrY), 0, 0, intrAlg);
  cv::resize(intMap, intrIntMap, cv::Size(intrX, intrY), 0, 0, intrAlg);

  float intrPitch = pitch_fov / intrRi.rows;
  float intrYaw   = yaw_fov / intrRi.cols;

  debug_print("pitch %f, yaw %f", pitch, yaw);

  riConverter.setConfig(min, max, intrPitch, intrYaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);
  types::PclPcXyzi intrXyzi = riConverter.reconstructPcFromRiWithIm(intrRi, intrIntMap, true);

  PcWriter pcWriter;
  pcWriter.writeBin(output, intrXyzi);

  ri.release();
  intMap.release();
  outRi.release();
  outIntMap.release();
  return 0;
}

