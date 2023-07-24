#include <downsampler.h>
#include <omp.h>

using namespace flicr;
using namespace std;

void RiDownSampler::downsample(vector<cv::Mat> inTiles, vector<vector<int>> inTileCount, float normFactor, int xt, int yt,
                               vector<vector<cv::Mat>> &outDownsampledTiles, vector<vector<types::DownsampledTile>> &outTileInfo)
{
  int tileTotal = inTiles[0].rows * inTiles[0].cols;

  float tileDegX = HDL64_HORIZONTAL_DEGREE / xt;
  float tileDegY = HDL64_VERTICAL_DEGREE / yt;

  outDownsampledTiles.resize(inTileCount.size());
  outTileInfo.resize(inTileCount.size());
  for(int i = 0; i < outDownsampledTiles.size(); i++)
  {
    outDownsampledTiles[i].resize(inTileCount[i].size());
    outTileInfo[i].resize(inTileCount[i].size());
  }

  for (int i = 0; i < inTiles.size(); i++)
  {
    int yCount = i / xt;
    int xCount = i % xt;

    float density = (float)inTileCount[yCount][xCount] / tileTotal / normFactor;

    float degOffsetX = tileDegX * xCount;
    float degOffsetY = tileDegY * yCount;

    int width = inTiles[i].cols * density;
    float yawPrec = tileDegX / (float)width;

    types::DownsampledTile t;
    t.set(yawPrec,
          HDL64_THETA_PRECISION,
          HDL64_HORIZONTAL_DEGREE_OFFSET - degOffsetX, // plus or minus..?
          HDL64_VERTICAL_DEGREE_OFFSET - degOffsetY);

    cv::Mat downsampledTile;
    cv::resize(inTiles[i], downsampledTile, cv::Size(width, inTiles[i].rows), 0, 0, cv::INTER_NEAREST);

    outDownsampledTiles[yCount][xCount] = downsampledTile;
    outTileInfo[yCount][xCount] = t;
  }
}

void RiDownSampler::downsample(vector<cv::Mat> inTiles, vector<cv::Mat> intMapTiles, vector<vector<int>> inTileCount, float normFactor, int xt, int yt,
                               vector<vector<cv::Mat>> &outDownsampledTiles,
                               vector<vector<cv::Mat>> &outDownsampledIntMapTile,
                               vector<vector<types::DownsampledTile>> &outTileInfo)
{
  int tileTotal = inTiles[0].rows * inTiles[0].cols;

  float tileDegX = HDL64_HORIZONTAL_DEGREE / xt;
  float tileDegY = HDL64_VERTICAL_DEGREE / yt;

  outDownsampledTiles.resize(inTileCount.size());
  outDownsampledIntMapTile.resize(inTileCount.size());
  outTileInfo.resize(inTileCount.size());
  for(int i = 0; i < outDownsampledTiles.size(); i++)
  {
    outDownsampledTiles[i].resize(inTileCount[i].size());
    outDownsampledIntMapTile[i].resize(inTileCount[i].size());
    outTileInfo[i].resize(inTileCount[i].size());
  }

  for (int i = 0; i < inTiles.size(); i++)
  {
    int yCount = i / xt;
    int xCount = i % xt;

    float density = (float)inTileCount[yCount][xCount] / tileTotal / normFactor;

    float degOffsetX = tileDegX * xCount;
    float degOffsetY = tileDegY * yCount;

    int width = inTiles[i].cols * density;
    float yawPrec = tileDegX / (float)width;

    types::DownsampledTile t;
    t.set(yawPrec,
          HDL64_THETA_PRECISION,
          HDL64_HORIZONTAL_DEGREE_OFFSET - degOffsetX, // plus or minus..?
          HDL64_VERTICAL_DEGREE_OFFSET - degOffsetY);

    cv::Mat downsampledTile, downsampledIntMapTile;
    cv::resize(inTiles[i], downsampledTile, cv::Size(width, inTiles[i].rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(intMapTiles[i], downsampledIntMapTile, cv::Size(width, inTiles[i].rows), 0, 0, cv::INTER_NEAREST);

    outDownsampledTiles[yCount][xCount] = downsampledTile;
    outDownsampledIntMapTile[yCount][xCount] = downsampledIntMapTile;
    outTileInfo[yCount][xCount] = t;
  }
}


types::PclPcXyzi RiDownSampler::RecPcFromTiledDwonsampledRi(vector<vector<cv::Mat>> downTiles,
                                                            vector<vector<cv::Mat>> downIntTiles,
                                                            vector<vector<types::DownsampledTile>> downTileInfo)
{
  types::PclPcXyzi pc(new pcl::PointCloud<types::PclXyzi>);
  pc->reserve(160000);

  for (int ty = 0; ty < downTiles.size(); ty++)
  {
    for (int tx = 0; tx < downTiles[0].size(); tx++)
    {
      cv::Mat tile = downTiles[ty][tx];
      cv::Mat intTile = downIntTiles[ty][tx];
      types::DownsampledTile tileInfo = downTileInfo[ty][tx];
      for (int y = 0; y < tile.rows; y++)
      {
        for (int x = 0; x <= tile.cols; x++)
        {
          float rho = tile.at<float>(y, x);
          float intensity = intTile.at<float>(y, x);

          types::PclXyzi p;
          TiledRtpToXyzi(tileInfo, rho, y, x, p.x, p.y, p.z);
          p.intensity = intensity;
          pc->insert(pc->end(), p);
        }
      }
    }
  }
  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


types::PclPcXyzi RiDownSampler::RecPcFromTiledDwonsampledRiParallel(vector<vector<cv::Mat>> downTiles,
                                                       vector<vector<cv::Mat>> downIntTiles,
                                                       vector<vector<types::DownsampledTile>> downTileInfo)
{
  types::PclPcXyzi pc(new pcl::PointCloud<types::PclXyzi>);
  pc->reserve(160000);

  // Private point cloud for each thread
  std::vector<types::PclPcXyzi> privatePcs(omp_get_max_threads());
  for (auto &privatePc : privatePcs)
  {
    privatePc.reset(new pcl::PointCloud<types::PclXyzi>);
    privatePc->reserve(40000);
  }

  #pragma omp parallel for collapse(2)
  for (int ty = 0; ty < downTiles.size(); ty++)
  {
    for (int tx = 0; tx < downTiles[0].size(); tx++)
    {
      cv::Mat tile = downTiles[ty][tx];
      cv::Mat intTile = downIntTiles[ty][tx];
      types::DownsampledTile tileInfo = downTileInfo[ty][tx];
      int threadId = omp_get_thread_num(); // Get the current thread ID

      for (int y = 0; y < tile.rows; y++)
      {
        for (int x = 0; x < tile.cols; x++)
        {
          float rho = tile.at<float>(y, x);
          float intensity = intTile.at<float>(y, x);

          types::PclXyzi p;
          TiledRtpToXyzi(tileInfo, rho, y, x, p.x, p.y, p.z); // Implement this function accordingly
          p.intensity = intensity;

          privatePcs[threadId]->push_back(p);
        }
      }
    }
  }

  // Merge private point clouds into the shared final point cloud
  for (const auto &privatePc : privatePcs)
  {
    pc->insert(pc->end(), privatePc->begin(), privatePc->end());
  }

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


void RiDownSampler::TiledRtpToXyzi(types::DownsampledTile tileInfo,
                                   float rho, int pitchRow, int yawCol,
                                   float &x, float &y, float &z)
{
  float dPitch = (pitchRow * tileInfo.pitchPrec) - tileInfo.pitchOffset;
  float dYaw = (yawCol * tileInfo.yawPrec) - tileInfo.yawOffset;

  float rPitch = DEGREE2RAD(dPitch);
  float rYaw   = DEGREE2RAD(dYaw);

  x = rho * std::sin(rPitch) * std::cos(rYaw);
  y = rho * std::sin(rPitch) * std::sin(rYaw);
  z = rho * std::cos(rPitch);
}

// types::PclPcXyzi RiConverter::reconstructPcFromRiWithIm(cv::Mat &ri, cv::Mat &intMap, bool parallel)
// {
//   types::PclPcXyzi pc(new pcl::PointCloud<types::PclXyzi>);
//   pc->reserve(160000);
//
//   #pragma omp parallel if (parallel)
//   {
//     pcl::PointCloud<types::PclXyzi> privatePc;
//     privatePc.reserve(40000);
//
//     #pragma omp for nowait
//     for(int y = 0; y <= ri.rows; y++) {
//       for(int x = 0; x <= ri.cols; x++) {
//       }
//     }
//
//     #pragma omp critical
//     pc->insert(pc->end(), privatePc.begin(), privatePc.end());
//   }
//
//   pc->width = pc->size();
//   pc->height = 1;
//   return pc;
// }
//
