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
  void downsample(vector<cv::Mat> inTiles, vector<int> inTileCount, int xt, int yt,
                  vector<cv::Mat> outDownsampledTiles, vector<types::DownsampledTile> outTileInfo);
};
}