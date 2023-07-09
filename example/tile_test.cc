#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("compressor", "FLiCR Compressor (XYZI->XYZ)");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("yaw_fov",   "yaw fov", cxxopts::value<float>())
    ("pitch_fov", "pitch fov", cxxopts::value<float>())
    ("yaw_offset",   "yaw offset", cxxopts::value<float>())
    ("pitch_offset", "pitch offset", cxxopts::value<float>())
    ("min_range", "minimum range", cxxopts::value<float>())
    ("max_range", "maximum range", cxxopts::value<float>())

    ("x, width", "range image width", cxxopts::value<int>())
    ("y, height", "range image height", cxxopts::value<int>())

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
  float yaw_fov   = parsedArgs["yaw_fov"].as<float>();
  float pitch_fov = parsedArgs["pitch_fov"].as<float>();
  float yaw_offset   = parsedArgs["yaw_offset"].as<float>();
  float pitch_offset = parsedArgs["pitch_offset"].as<float>();
  float min   = parsedArgs["min_range"].as<float>();
  float max   = parsedArgs["max_range"].as<float>();
  int x = parsedArgs["width"].as<int>();
  int y = parsedArgs["height"].as<int>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput file: " << input << endl;
    cout << "\tsensor's yaw FoV: "   << yaw_fov << endl;
    cout << "\tsensor's pitch FoV: " << pitch_fov << endl;
    cout << "\tsensor's yaw offset: "   << yaw_offset << endl;
    cout << "\tsensor's pitch offset: " << pitch_offset << endl;
    cout << "\tsensor's min/max range: " << min << ", " << max << endl;
    cout << "\tRI, intMap resolution: " << x << ", " << y << endl;
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
  double riMin, riMax;
  double intMapMin, intMapMax;

  riConverter.convertPc2RiWithIm(xyzi, ri, intMap, true);

  Tiler tiler;
  // std::vector<cv::Mat> split(cv::Mat image, int x_tiles, int y_tiles);
  std::vector<cv::Mat> tiles = tiler.split(ri, 2, 2);

  // change pixel value of ri
  cout << "[BEFORE]" << endl;
  cout << ri.at<float>(10, 10) << endl;
  cout << tiles[0].at<float>(10, 10) << endl;

  ri.at<float>(10, 10) = 444;

  cout << "[AFTER]" << endl;
  cout << ri.at<float>(10, 10) << endl;
  cout << tiles[0].at<float>(10, 10) << endl;

  ri.release();
  intMap.release();
  return 0;
}

