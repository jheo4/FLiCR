#include "opencv2/imgcodecs.hpp"
#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("utils", "Histogram");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())

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

  float yaw_fov   = HDL64_HORIZONTAL_DEGREE;
  float pitch_fov = HDL64_VERTICAL_DEGREE;
  float yaw_offset   = HDL64_HORIZONTAL_DEGREE_OFFSET;
  float pitch_offset = HDL64_VERTICAL_DEGREE_OFFSET;
  float min   = 0;
  float max   = 80;

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
    cout << "\tX/Y tile nums: " << xt << ", " << yt << endl;
  }

  PcReader pcReader;
  types::PclPcXyzi xyzi = NULL;
  vector<float> points;

  float pitch = HDL64_THETA_PRECISION;
  float yaw   = HDL64_PI_PRECISION_4500;
  float avgMSE = 0, avgPSNR = 0;

  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  // Testing...
  xyzi = pcReader.readXyziBin(input);
  if (xyzi == NULL)
  {
    if (debug) debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }

  // PC -> tiled RI
  cv::Mat ri, intMap, copiedRi;
  vector<cv::Mat> riTiles;
  vector<cv::Mat> intMapTiles;
  vector<vector<int>> tileCount;

  riConverter.PcToRiImWithTile(xyzi, xt, yt, ri, intMap, riTiles, intMapTiles, tileCount, true);

  for(int y = 0; y < tileCount.size(); y++)
    for(int x = 0; x < tileCount[0].size(); x++)
    {
      cout << "tile " << y << ", " << x << " count: " << tileCount[y][x] << endl;
    }

  // copiedRi = ri.clone();

  vector<cv::Mat> normTiles;
  vector<pair<float, float>> tileMinMax;

  // iterate tile and normalize the RI tiles
  for (int i = 0; i < riTiles.size(); i++)
  {
    cv::Mat tile = riTiles[i];
    cv::Mat normTile;
    double min, max;
    riConverter.normalizeRi(tile, normTile, min, max);
    cout << i << "th  min: " << min << ", max: " << max << endl;
    normTiles.push_back(normTile);
    tileMinMax.push_back({min, max});
  }

  /*
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
  */

  return 0;
}

