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


  // 1. read PC
  xyzi = pcReader.readXyziBin(input);
  if (xyzi == NULL)
  {
    if (debug) debug_print("reading input file (%s) failed..", input.c_str());
    exit(1);
  }

  // 2. convert PC to RI
  cv::Mat ri, intMap, copiedRi;
  vector<cv::Mat> riTiles;
  vector<cv::Mat> intMapTiles;
  vector<vector<int>> tileCount;

  riConverter.PcToRiImWithTile(xyzi, xt, yt, ri, intMap, riTiles, intMapTiles, tileCount, true);

  for(int y = 0; y < tileCount.size(); y++)
  {
    for(int x = 0; x < tileCount[0].size(); x++)
    {
      cout << "tile " << y << ", " << x << " count: " << tileCount[y][x] << endl;
    }
  }

  // 3. downsample RI tiles
  RiDownSampler riDownSampler;
  vector<vector<cv::Mat>> downsampledTiles;
  vector<vector<cv::Mat>> downsampledIntMapTiles;
  vector<vector<types::DownsampledTile>> downsampledTileInfo;
  riDownSampler.downsample(riTiles, intMapTiles, tileCount, xt, yt, downsampledTiles, downsampledIntMapTiles, downsampledTileInfo);

  for(int i = 0; i < downsampledTileInfo.size(); i++)
  {
    for(int j = 0; j < downsampledTileInfo[i].size(); j++)
    {
      downsampledTileInfo[i][j].print(i*yt+j);
    }

  }


  // 4. quantize downsampled RI tiles
  vector<vector<pair<float, float>>> tileMinMax, intMapMinMax;
  tileMinMax.resize(downsampledTiles.size());
  intMapMinMax.resize(downsampledTiles.size());
  for(int x = 0; x < downsampledTiles.size(); x++)
  {
    tileMinMax[x].resize(downsampledTiles[x].size());
    intMapMinMax[x].resize(downsampledTiles[x].size());
  }

  for(int y = 0; y < downsampledTiles.size(); y++)
  {
    for(int x = 0; x < downsampledTiles[y].size(); x++)
    {
      cv::Mat tile = downsampledTiles[y][x];
      cv::Mat intTile = downsampledIntMapTiles[y][x];
      double min, max, intMin, intMax;
      riConverter.normalizeRi(tile, tile, min, max);
      riConverter.normalizeRi(intTile, intTile, intMin, intMax);

      tileMinMax[y][x] = make_pair(min, max);
      intMapMinMax[y][x] = make_pair(intMin, intMax);
    }
  }

  // 5. denormalize downsampled RI tiles
  vector<vector<cv::Mat>> denormTiles;
  vector<vector<cv::Mat>> denormIntMapTiles;
  denormTiles.resize(downsampledTiles.size());
  denormIntMapTiles.resize(downsampledTiles.size());
  for(int x = 0; x < downsampledTiles.size(); x++)
  {
    denormTiles[x].resize(downsampledTiles[x].size());
    denormIntMapTiles[x].resize(downsampledTiles[x].size());
  }

  for(int y = 0; y < downsampledTiles.size(); y++)
  {
    for(int x = 0; x < downsampledTiles[y].size(); x++)
    {
      cv::Mat tile = downsampledTiles[y][x];
      cv::Mat intTile = downsampledIntMapTiles[y][x];
      double min = tileMinMax[y][x].first;
      double max = tileMinMax[y][x].second;
      double intMin = intMapMinMax[y][x].first;
      double intMax = intMapMinMax[y][x].second;
      cout << "tile " << y << ", " << x << " min: " << min << ", max: " << max << endl;
      cout << "intMap tile " << y << ", " << x << " min: " << intMin << ", max: " << intMax << endl;
      riConverter.denormalizeRi(tile, min, max, denormTiles[y][x]);
      riConverter.denormalizeRi(intTile, intMin, intMax, denormIntMapTiles[y][x]);
    }
  }

  // 6. reconstruct PC from denormalized RI tiles
  // types::PclPcXyzi pc = riDownSampler.RecPcFromTiledDwonsampledRi(downsampledTiles, downsampledIntMapTiles, downsampledTileInfo);
  types::PclPcXyzi pc = riDownSampler.RecPcFromTiledDwonsampledRiParallel(downsampledTiles, downsampledIntMapTiles, downsampledTileInfo);
  types::PclPcXyz xyz = types::xyzi2xyz(pc);
  types::PclPcXyz origXyz = types::xyzi2xyz(xyzi);

  Visualizer visualizer1;
  visualizer1.initViewerXyz();
  visualizer1.setViewer(origXyz);
  visualizer1.show(5000);

  Visualizer visualizer2;
  visualizer2.initViewerXyz();
  visualizer2.setViewer(xyz);
  visualizer2.show(5000);

  return 0;
}

