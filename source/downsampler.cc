#include <downsampler.h>

using namespace flicr;
using namespace std;

void RiDownSampler::downsample(vector<cv::Mat> inTiles, vector<vector<int>> inTileCount, int xt, int yt,
                               vector<cv::Mat> &outDownsampledTiles, vector<types::DownsampledTile> &outTileInfo)
{
  int tileTotal = inTiles[0].rows * inTiles[0].cols;

  int tileDegX = HDL64_HORIZONTAL_DEGREE / xt;
  int tileDegY = HDL64_VERTICAL_DEGREE / yt;

  int rowDegIndex = 0;
  int colDegIndex = 0;

  for (int i = 0; i < inTiles.size(); i++)
  {
    int yCount = i / xt;
    int xCount = i % xt;

    float density = (float)inTileCount[yCount][xCount] / tileTotal;

    float degOffsetX = tileDegX * colDegIndex;
    float degOffsetY = tileDegY * rowDegIndex;

    colDegIndex = i % xt;
    rowDegIndex = i / xt;

    int x = inTiles[i].cols * density;
    float yawPrec = tileDegX / x;

    types::DownsampledTile t;
    t.set(yawPrec,
          HDL64_THETA_PRECISION,
          HDL64_HORIZONTAL_DEGREE_OFFSET - degOffsetX,
          HDL64_VERTICAL_DEGREE_OFFSET - degOffsetY);

    cv::Mat downsampledTile;
    cv::resize(inTiles[i], downsampledTile, cv::Size(x, inTiles[i].rows), 0, 0, cv::INTER_NEAREST);

    outDownsampledTiles.push_back(downsampledTile);
    outTileInfo.push_back(t);
  }
}
