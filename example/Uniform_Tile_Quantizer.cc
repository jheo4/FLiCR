#include "opencv2/imgcodecs.hpp"
#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("utils", "Histogram");
  options.add_options()
    ("tt", "testing data directory", cxxopts::value<std::string>())
    ("tt_num", "num of testing dataset", cxxopts::value<int>())

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

  std::string testing_path  = parsedArgs["tt"].as<std::string>();
  int testing_data_num = parsedArgs["tt_num"].as<int>();

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
    cout << "\ttesting data path: " << testing_path << ", testing data num: " << testing_data_num << endl;
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
  float avgMSE = 0, avgPSNR = 0, avgRMSE = 0;

  RiConverter riConverter;
  riConverter.setConfig(0, 80, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  // Testing...
  for (int i = 0; i < testing_data_num; i++)
  {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << i;
    std::string file_name = ss.str();

    std::string input = testing_path + "/" + file_name + ".bin";
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
      normTiles.push_back(normTile);
      tileMinMax.push_back({min, max});
    }

    // iterate the normalized tiles and denormalize...
    cv::Mat denormRi = cv::Mat::zeros(ri.rows, ri.cols, CV_32FC1);
    vector<cv::Mat> denormTiles = tiler.split(denormRi, xt, yt);

    for (int i = 0; i < normTiles.size(); i++)
    {
      double min = tileMinMax[i].first;
      double max = tileMinMax[i].second;
      riConverter.denormalizeRi(normTiles[i], min, max, denormTiles[i]);

      float tileMSE, tilePSNR;
      tileMSE  = Metrics::calculateMSE(tiles[i], denormTiles[i]);
      tilePSNR = 10 * log10(pow(80, 2) / tileMSE);

      cout << i << "th tileMSE: " << tileMSE << ", tilePSNR: " << tilePSNR << endl;
    }

    float MSE, PSNR;
    MSE = Metrics::calculateMSE(ri, denormRi);
    PSNR = 10 * log10(pow(80, 2) / MSE);

    avgMSE  += MSE;
    avgRMSE += sqrt(MSE);
    avgPSNR += PSNR;

    ri.release();
    denormRi.release();
    copiedRi.release();
    intMap.release();

  }

  avgMSE /= testing_data_num;
  avgPSNR /= testing_data_num;
  cout << "[TEST] avgMSE: " << avgMSE << ", avgRMSE: " << avgRMSE << ", avgPSNR: " << avgPSNR << endl;

  return 0;
}

