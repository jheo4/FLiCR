#include "opencv2/imgcodecs.hpp"
#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("utils", "Histogram");
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

    ("xt", "x-wise tiles", cxxopts::value<int>())
    ("yt", "y-wise tiles", cxxopts::value<int>())

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
  int xt = parsedArgs["xt"].as<int>();
  int yt = parsedArgs["yt"].as<int>();
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
    cout << "\tX/Y tile nums: " << xt << ", " << yt << endl;
  }

  PcReader pcReader;
  types::PclPcXyzi xyzi = NULL;
  vector<float> points;

  float pitch = pitch_fov / y;
  float yaw   = yaw_fov / x;
  float avgMSE = 0, avgPSNR = 0;

  RiConverter riConverter;
  riConverter.setConfig(0, 80, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  // Testing...
  xyzi = pcReader.readXyziBin(input);
  if (xyzi == NULL)
  {
    if (debug)
      debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }

  // PC -> RI
  cv::Mat ri, intMap, copiedRi;
  riConverter.convertPc2RiWithIm(xyzi, ri, intMap, true);


  copiedRi = ri.clone();

  // RI -> Tiled RIs
  Tiler tiler;
  vector<cv::Mat> tiles = tiler.split(ri, xt, yt);


  vector<cv::Mat> normTiles;
  vector<pair<float, float>> tileMinMax;

  // iterate tile and normalize the RI tiles
  for (int i = 0; i < tiles.size(); i++)
  {
    cv::Mat tile = tiles[i];
    cv::Mat normTile;
    double min, max;
    riConverter.normalizeRi(tile, normTile, min, max);
    cout << i << "th  min: " << min << ", max: " << max << endl;
    normTiles.push_back(normTile);
    tileMinMax.push_back({min, max});
  }

  // create cvMat with same size of ri
  cv::Mat denormRi = cv::Mat::zeros(ri.rows, ri.cols, CV_32FC1);
  vector<cv::Mat> denormTiles = tiler.split(denormRi, xt, yt);

  float MSE = 0, PSNR = 0;
  // iterate the normalized tiles and denormalize...
  for (int i = 0; i < normTiles.size(); i++)
  {
    double min = tileMinMax[i].first;
    double max = tileMinMax[i].second;
    cout << i << "th  min: " << min << ", max: " << max << endl;
    riConverter.denormalizeRi(normTiles[i], min, max, denormTiles[i]);

    float tileMSE, tilePSNR;
    tileMSE  = Metrics::calculateMSE(tiles[i], denormTiles[i]);
    tilePSNR = 10 * log10(pow(80, 2) / tileMSE);

    MSE  += tileMSE;
    PSNR += tilePSNR;

    cout << i << "th tileMSE: " << tileMSE << ", tilePSNR: " << tilePSNR << endl;

  }

  MSE  /= normTiles.size();
  PSNR /= normTiles.size();

  float totalMSE = Metrics::calculateMSE(ri, denormRi);
  float totalPSNR = 10 * log10(pow(80, 2) / totalMSE);

  ri.release();
  copiedRi.release();
  intMap.release();

  cout << "[TEST] avgMSE: " << MSE << ", avgPSNR: " << PSNR << endl;
  cout << "[TEST] totalMSE: " << totalMSE << ", totalPSNR: " << totalPSNR << endl;

  return 0;
}

