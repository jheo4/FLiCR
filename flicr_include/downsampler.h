#pragma once

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <types.h>

using namespace std;

namespace flicr
{
class RiDownSampler
{
public:
  void downsample(vector<cv::Mat> inTiles, vector<vector<int>> inTileCount, float normFactor, int xt, int yt,
                  vector<vector<cv::Mat>> &outDownsampledTiles, vector<vector<types::DownsampledTile>> &outTileInfo);

  void downsample(vector<cv::Mat> inTiles, vector<cv::Mat> intMapTiles, vector<vector<int>> inTileCount, float normFactor, int xt, int yt,
                  vector<vector<cv::Mat>> &outDownsampledTiles, vector<vector<cv::Mat>> &outDownsampledIntMapTile, vector<vector<types::DownsampledTile>> &outTileInfo);



  types::PclPcXyzi RecPcFromTiledDwonsampledRi(vector<vector<cv::Mat>> downTiles,
                                               vector<vector<cv::Mat>> downIntTiles,
                                               vector<vector<types::DownsampledTile>> downTileInfo);
  types::PclPcXyzi RecPcFromTiledDwonsampledRiParallel(vector<vector<cv::Mat>> downTiles,
                                                       vector<vector<cv::Mat>> downIntTiles,
                                                       vector<vector<types::DownsampledTile>> downTileInfo);

  void TiledRtpToXyzi(types::DownsampledTile tileInfo,
                      float rho, int pitchRow, int yawCol, float &x, float &y, float &z);
};
}