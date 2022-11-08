#include "opencv2/imgproc.hpp"
#include <flicr>

#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

using namespace std;
using namespace flicr;


int main(int argc, char **argv)
{
  cxxopts::Options options("FLiCR", "FLiCR");
  options.add_options()
    ("i, inri", "Input RI", cxxopts::value<std::string>())
    ("m, inintmap", "Input intensity map", cxxopts::value<std::string>())
    ("n, normalized", "normalized ri/intmap", cxxopts::value<bool>())

    ("x, intr_x", "width of interpolated ri", cxxopts::value<int>())
    ("y, intr_y", "height of interpolated ri", cxxopts::value<int>())
    ("a, algorithm", "CV algorithm to use [linear, cubic, lz4, area, nearest]", cxxopts::value<std::string>())

    ("o, outpc", "Output PC (xyzi)", cxxopts::value<std::string>())
    ("yaw_fov",   "yaw fov", cxxopts::value<float>())
    ("pitch_fov", "pitch fov", cxxopts::value<float>())
    ("yaw_offset",   "yaw offset", cxxopts::value<float>())
    ("pitch_offset", "pitch offset", cxxopts::value<float>())
    ("min_range", "minimum range", cxxopts::value<float>())
    ("max_range", "maximum range", cxxopts::value<float>())

    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string inRi  = parsedArgs["inri"].as<std::string>();
  std::string inIntMap  = parsedArgs["inintmap"].as<std::string>();
  bool norm = parsedArgs["normalized"].as<bool>();

  int intrX = parsedArgs["intr_x"].as<int>();
  int intrY = parsedArgs["intr_y"].as<int>();
  std::string algorithm = parsedArgs["algorithm"].as<std::string>();

  std::string output = parsedArgs["outpc"].as<std::string>();
  float yaw_fov   = parsedArgs["yaw_fov"].as<float>();
  float pitch_fov = parsedArgs["pitch_fov"].as<float>();
  float yaw_offset   = parsedArgs["yaw_offset"].as<float>();
  float pitch_offset = parsedArgs["pitch_offset"].as<float>();
  float min   = parsedArgs["min_range"].as<float>();
  float max   = parsedArgs["max_range"].as<float>();

  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput RI: " << inRi << endl;
    cout << "\tinput intensity map: " << inIntMap << endl;
    cout << "\tRI/IntMap Normalized?: " << norm << endl;

    cout << "\tCV algorithm: " << algorithm  << ", intRes: " << intrX << ", " << intrY << endl;

    cout << "\toutput file: " << output << endl;
    cout << "\tsensor's yaw FoV: "   << yaw_fov << endl;
    cout << "\tsensor's pitch FoV: " << pitch_fov << endl;
    cout << "\tsensor's yaw offset: "   << yaw_offset << endl;
    cout << "\tsensor's pitch offset: " << pitch_offset << endl;
    cout << "\tsensor's min/max range: " << min << ", " << max << endl;
  }

  cv::Mat inputRi, inputIntMap;
  cv::Mat ri, intMap;
  cv::Mat intrRi, intrIntMap;

  if(norm)
  {
    inputRi = cv::imread(inRi, CV_8UC1);
    inputIntMap = cv::imread(inIntMap, CV_8UC1);

    cv::normalize(inputRi, ri, min, max, cv::NORM_MINMAX, CV_32FC1);
    cv::normalize(inputIntMap, intMap, 0, 255, cv::NORM_MINMAX, CV_32FC1);
  }
  else
  {
    cv::FileStorage riFile(inRi, cv::FileStorage::READ);
    riFile["ri"] >> ri;
    riFile.release();

    cv::FileStorage intmapFile(inIntMap, cv::FileStorage::READ);
    intmapFile["intmap"] >> intMap;
    intmapFile.release();
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

  float pitch = pitch_fov / intrRi.rows;
  float yaw   = yaw_fov / intrRi.cols;

  debug_print("pitch %f, yaw %f", pitch, yaw);

  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);
  types::PclPcXyzi intrXyzi = riConverter.reconstructPcFromRiWithIm(intrRi, intrIntMap, true);

  PcWriter pcWriter;
  pcWriter.writeBin(output, intrXyzi);
  return 0;
}

