#include <ri_converter.h>
#include <tiler.h>
#include <defs.h>

using namespace flicr;

RiConverter::RiConverter(double minRange,          double maxRange,
                         double pitchPrecision,    double yawPrecision,
                         double pitchDegree,       double yawDegree,
                         double pitchDegreeOffset, double yawDegreeOffset)
{
  setConfig(minRange, maxRange, pitchPrecision, yawPrecision, pitchDegree, yawDegree, pitchDegreeOffset, yawDegreeOffset);
}


void RiConverter::setConfig(double minRange,          double maxRange,
                            double pitchPrecision,    double yawPrecision,
                            double pitchDegree,       double yawDegree,
                            double pitchDegreeOffset, double yawDegreeOffset)
{
  // Min-Max range of a sensor
  this->minRange             = minRange;
  this->maxRange             = maxRange;

  // Pitch (y, row) precision, Yaw (x, col) precision
  this->pitchPrecision       = pitchPrecision;
  this->yawPrecision          = yawPrecision;

  // Pitch degree: Y sensing degree
  // Yaw degree   : X sensing degree
  this->pitchDegree          = pitchDegree;
  this->yawDegree             = yawDegree;

  // Pitch (y) degree offset
  // Yaw (x) degree offset
  this->pitchDegreeOffset    = pitchDegreeOffset;
  this->yawDegreeOffset       = yawDegreeOffset;

  this->riRow = pitchDegree / pitchPrecision;
  this->riCol = yawDegree    / yawPrecision;
}


void RiConverter::setResolution(int row, int col)
{
  this->pitchPrecision = (double)this->pitchDegree / row;
  this->yawPrecision = (double)this->yawDegree / col;
  this->riRow = row;
  this->riCol = col;
}


void RiConverter::XYZ2RTP(float &x, float &y, float &z, float &rho, int &pitchRow, int &yawCol)
{
  rho    = std::sqrt(x*x + y*y + z*z);
  pitchRow = (int)((RAD2DEGREE(std::acos(z / rho)) + pitchDegreeOffset) / pitchPrecision);
  yawCol = (int)((RAD2DEGREE(std::atan2(y, x)) + yawDegreeOffset) / yawPrecision);
}


void RiConverter::RTP2XYZ(float &rho, int &pitchRow, int &yawCol, float &x, float &y, float &z)
{
  float dPitch = (pitchRow * pitchPrecision) - pitchDegreeOffset;
  float dYaw = (yawCol * yawPrecision) - yawDegreeOffset;

  float rPitch = DEGREE2RAD(dPitch);
  float rYaw   = DEGREE2RAD(dYaw);

  x = rho * std::sin(rPitch) * std::cos(rYaw);
  y = rho * std::sin(rPitch) * std::sin(rYaw);
  z = rho * std::cos(rPitch);
}


void RiConverter::convertRawPc2Ri(types::RawPc inPc, cv::Mat &outRi, bool parallel)
{
  outRi = cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f));

  #pragma omp parallel for if (parallel)
  for(uint32_t i = 0; i < inPc.numOfPoints; i++)
  {
    int pointIdx = i*3;
    float x = inPc.buf[pointIdx];
    float y = inPc.buf[pointIdx+1];
    float z = inPc.buf[pointIdx+2];

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));

    if(outRi.at<float>(rowIdx, colIdx) == 0)
      outRi.at<float>(rowIdx, colIdx) = rho;
    else if(rho > outRi.at<float>(rowIdx, colIdx))
      outRi.at<float>(rowIdx, colIdx) = rho;
  }
}


void RiConverter::PcToRiWithTile(types::PclPcXyz inPc, int xt, int yt, cv::Mat &outRi, vector<cv::Mat> &outTiles, vector<vector<int>> &outTileCount, bool parallel)
{
  outRi = cv::Mat(this->riRow, this->riCol, CV_32FC1, cv::Scalar(0.f));

  outTileCount.resize(yt);
  for(int i = 0; i < yt; i++) outTileCount[i].resize(xt, 0);

  #pragma omp parallel for if (parallel)
  for(int i = 0; i < (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));


    if(outRi.at<float>(rowIdx, colIdx) == 0)
    {
      outRi.at<float>(rowIdx, colIdx) = rho;
      int tileX = (int)(colIdx / xt);
      int tileY = (int)(rowIdx / yt);
      outTileCount[tileY][tileX]++;
    }
  }

  outTiles = this->tiler.split(outRi, riCol/xt, riRow/yt);
}

void RiConverter::PcToRiImWithTile(types::PclPcXyzi inPc, int xt, int yt,
                                   cv::Mat &outRi, cv::Mat &outIm, vector<cv::Mat> &outRiTiles, vector<cv::Mat> &outImTiles,
                                   vector<vector<int>> &outTileCount, bool parallel)
{
  outRi = cv::Mat(this->riRow, this->riCol, CV_32FC1, cv::Scalar(0.f));
  outIm = cv::Mat(this->riRow, this->riCol, CV_32FC1, cv::Scalar(0.f));

  outTileCount.resize(yt);
  for(int i = 0; i < yt; i++) outTileCount[i].resize(xt, 0);

  int tileSizeX = riCol/xt;
  int tileSizeY = riRow/yt;

  #pragma omp parallel for if (parallel)
  for(int i = 0; i< (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));

    if(outRi.at<float>(rowIdx, colIdx) == 0)
    {
      outRi.at<float>(rowIdx, colIdx) = rho;
      outIm.at<float>(rowIdx, colIdx) = inPc->points[i].intensity;
      int tileX = (int)(colIdx / tileSizeX);
      int tileY = (int)(rowIdx / tileSizeY);
      outTileCount[tileY][tileX]++;
    }
  }

  outRiTiles = this->tiler.split(outRi, xt, yt);
  outImTiles = this->tiler.split(outIm, xt, yt);
}

void RiConverter::convertPc2Ri(types::PclPcXyz inPc, cv::Mat &outRi, bool parallel)
{
  outRi = cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f));

  #pragma omp parallel for if (parallel)
  for(int i = 0; i < (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));

    if(outRi.at<float>(rowIdx, colIdx) == 0)
      outRi.at<float>(rowIdx, colIdx) = rho;
  }
}


void RiConverter::convertPc2RiWithI(types::PclPcXyzi inPc, cv::Mat &outRi, bool parallel)
{
  outRi = cv::Mat(riRow, riCol, CV_32FC2, cv::Scalar(0.f));

  #pragma omp parallel for if (parallel)
  for(int i = 0; i < (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));

    if(outRi.at<cv::Vec2f>(rowIdx, colIdx)[0] == 0)
    {
      outRi.at<cv::Vec2f>(rowIdx, colIdx)[0] = rho;
      outRi.at<cv::Vec2f>(rowIdx, colIdx)[1] = inPc->points[i].intensity;
    }
  }
}


void RiConverter::convertPc2RiWithIm(types::PclPcXyzi inPc, cv::Mat &outRi, cv::Mat &outIntMap, bool parallel)
{
  outRi     = cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f));
  outIntMap = cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f));

  #pragma omp parallel for if (parallel)
  for(int i = 0; i< (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, yawCol));

    if(outRi.at<float>(rowIdx, colIdx) == 0)
    {
      outRi.at<float>(rowIdx, colIdx) = rho;
      outIntMap.at<float>(rowIdx, colIdx) = inPc->points[i].intensity;
    }
  }
}


void RiConverter::convertPc2RiWithXyz(types::PclPcXyz inPc, cv::Mat &outRiWithXyz, bool parallel)
{
  outRiWithXyz = cv::Mat(riRow, riCol, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));

  #pragma omp parallel for if (parallel)
  for(int i = 0; i < (int)inPc->points.size(); i++)
  {
    float x = inPc->points[i].x;
    float y = inPc->points[i].y;
    float z = inPc->points[i].z;

    //float r = p.r; // need to be encoded?

    float rho;
    int pitchRow, yawCol;

    XYZ2RTP(x, y, z, rho, pitchRow, yawCol);

    int rowIdx = std::min(outRiWithXyz.rows-1, std::max(0, pitchRow));
    int colIdx = std::min(outRiWithXyz.cols-1, std::max(0, yawCol));

    outRiWithXyz.at<cv::Vec4f>(rowIdx, colIdx) = cv::Vec4f(rho, x, y, z);
  }
}


types::PclPcXyz RiConverter::reconstructPcFromRi(cv::Mat &ri, bool parallel)
{
  types::PclPcXyz pc(new pcl::PointCloud<types::PclXyz>);
  pc->reserve(160000);

  #pragma omp parallel if (parallel)
  {
    pcl::PointCloud<types::PclXyz> privatePc;
    privatePc.reserve(40000);

    #pragma omp for nowait
    for(int y = 0; y < ri.rows; y++) {
      for(int x = 0; x < ri.cols; x++) {
        float rho = ri.at<float>(y, x);
        if(rho > minRange && rho < maxRange) {
          types::PclXyz p;
          RTP2XYZ(rho, y, x, p.x, p.y, p.z);
          privatePc.push_back(p);
        }
      }
    }

    #pragma omp critical
    pc->insert(pc->end(), privatePc.begin(), privatePc.end());
  }

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


types::PclPcXyzi RiConverter::reconstructPcFromRiWithI(cv::Mat &ri, bool parallel)
{
  types::PclPcXyzi pc(new pcl::PointCloud<types::PclXyzi>);
  pc->reserve(160000);

  #pragma omp parallel if (parallel)
  {
    pcl::PointCloud<types::PclXyzi> privatePc;
    privatePc.reserve(40000);

    #pragma omp for nowait
    for(int y = 0; y <= ri.rows; y++) {
      for(int x = 0; x <= ri.cols; x++) {
        float rho       = ri.at<cv::Vec2f>(y, x)[0];
        float intensity = ri.at<cv::Vec2f>(y, x)[1];
        if(rho > minRange && rho < maxRange) {
          types::PclXyzi p;
          RTP2XYZ(rho, y, x, p.x, p.y, p.z);
          p.intensity = intensity;
          privatePc.push_back(p);
        }
      }
    }

    #pragma omp critical
    pc->insert(pc->end(), privatePc.begin(), privatePc.end());
  }

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


types::PclPcXyzi RiConverter::reconstructPcFromRiWithIm(cv::Mat &ri, cv::Mat &intMap, bool parallel)
{
  if(ri.rows != intMap.rows || ri.cols != intMap.cols)
  {
    debug_print("RI and IntMap are not matched.");
    return NULL;
  }

  types::PclPcXyzi pc(new pcl::PointCloud<types::PclXyzi>);
  pc->reserve(160000);

  #pragma omp parallel if (parallel)
  {
    pcl::PointCloud<types::PclXyzi> privatePc;
    privatePc.reserve(40000);

    #pragma omp for nowait
    for(int y = 0; y <= ri.rows; y++) {
      for(int x = 0; x <= ri.cols; x++) {
        float rho       = ri.at<float>(y, x);
        float intensity = intMap.at<float>(y, x);
        if(rho > minRange && rho < maxRange) {
          types::PclXyzi p;
          RTP2XYZ(rho, y, x, p.x, p.y, p.z);
          p.intensity = intensity;
          privatePc.push_back(p);
        }
      }
    }

    #pragma omp critical
    pc->insert(pc->end(), privatePc.begin(), privatePc.end());
  }

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


void RiConverter::normalizeRi(cv::Mat &origRi, cv::Mat &normRi)
{
  cv::normalize(origRi, normRi, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}


void RiConverter::normalizeRi(cv::Mat &origRi, cv::Mat &normRi, double &minRho, double &maxRho)
{
  cv::Point minP, maxP;
  cv::minMaxLoc(origRi, &minRho, &maxRho, &minP, &maxP);
  cv::normalize(origRi, normRi, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}


void RiConverter::normalizeRi(cv::Mat &origRi, cv::Mat &normRi, double &maxRho)
{
  double min;
  normalizeRi(origRi, normRi, min, maxRho);
}


void RiConverter::normalizeRiWithI(cv::Mat &origRiWithI, cv::Mat &normRiWithI, double &maxRho, double &maxInt)
{
  double minRho, minInt;
  cv::Point minP, maxP;

  cv::Mat riChannels[2];
  cv::split(origRiWithI, riChannels);
  cv::minMaxLoc(riChannels[0], &minRho, &maxRho, &minP, &maxP);
  cv::minMaxLoc(riChannels[1], &minInt, &maxInt, &minP, &maxP);
  //debug_print("rho/intensity range: %f~%f / %f~%f", minRho, *maxRho, minInt, *maxInt);

  cv::Mat normalizedChannels[2];
  cv::normalize(riChannels[0], normalizedChannels[0], 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::normalize(riChannels[1], normalizedChannels[1], 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::merge(normalizedChannels, 2, normRiWithI);
}


void RiConverter::normalizeRiWithI(cv::Mat &origRiWithI, cv::Mat &normRi, cv::Mat &intMap, double &maxRho, double &maxInt)
{
  double minRho, minInt;
  cv::Point minP, maxP;

  cv::Mat riChannels[2];
  cv::split(origRiWithI, riChannels);
  cv::minMaxLoc(riChannels[0], &minRho, &maxRho, &minP, &maxP);
  cv::minMaxLoc(riChannels[1], &minInt, &maxInt, &minP, &maxP);

  cv::normalize(riChannels[0], normRi, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::normalize(riChannels[1], intMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}

void RiConverter::quantizeMat(cv::Mat &origMat, int bytes, cv::Mat &outMat, double &outMin, double &outMax)
{
  cv::Point minP, maxP;
  cv::minMaxLoc(origMat, &outMin, &outMax, &minP, &maxP);

  if(bytes == 1)
    cv::normalize(origMat, outMat, 0, 0xff, cv::NORM_MINMAX, CV_8UC1);
  else if(bytes == 2)
    cv::normalize(origMat, outMat, 0, 0xffff, cv::NORM_MINMAX, CV_16UC1);
  else if(bytes == 3)
    cv::normalize(origMat, outMat, 0x00000000, 0x00ffffff, cv::NORM_MINMAX, CV_32SC1);
  else if(bytes == 4)
    cv::normalize(origMat, outMat, 0x00000000, 0xffffffff, cv::NORM_MINMAX, CV_32SC1);
  else if(bytes == 5)
    cv::normalize(origMat, outMat, 0x0000000000000000, 0x000000ffffffffff, cv::NORM_MINMAX, CV_64FC1);
  else if(bytes == 6)
    cv::normalize(origMat, outMat, 0x0000000000000000, 0x0000ffffffffffff, cv::NORM_MINMAX, CV_64FC1);
  else if(bytes == 7)
    cv::normalize(origMat, outMat, 0x0000000000000000, 0x00ffffffffffffff, cv::NORM_MINMAX, CV_64FC1);
  else
    cv::normalize(origMat, outMat, 0x0000000000000000, 0xffffffffffffffff, cv::NORM_MINMAX, CV_64FC1);
}

void RiConverter::dequantizeMat(cv::Mat &quantMat, double min, double max, cv::Mat &dequantMat)
{
  cv::normalize(quantMat, dequantMat, min, max, cv::NORM_MINMAX, CV_32FC1);
}

void RiConverter::denormalizeRi(cv::Mat &normRi, double minRho, double maxRho, cv::Mat &denormRi)
{
  cv::normalize(normRi, denormRi, minRho, maxRho, cv::NORM_MINMAX, CV_32FC1);
}


void RiConverter::denormalizeRiWithI(cv::Mat &normRiWithI, double maxRho, double maxInt, cv::Mat &denormRiWithI)
{
  cv::Mat normalizedChannels[2], denormalizedChannels[2];
  cv::split(normRiWithI, normalizedChannels);

  cv::normalize(normalizedChannels[0], denormalizedChannels[0], 0, maxRho, cv::NORM_MINMAX, CV_32FC1);
  cv::normalize(normalizedChannels[1], denormalizedChannels[1], 0, maxInt, cv::NORM_MINMAX, CV_32FC1);

  cv::merge(denormalizedChannels, 2, denormRiWithI);
}


void RiConverter::denormalizeRiWithI(cv::Mat &normRi, cv::Mat &intMap, double maxRho, double maxInt, cv::Mat &denormRiWithI)
{
  cv::Mat denormalizedChannels[2];

  cv::normalize(normRi, denormalizedChannels[0], 0, maxRho, cv::NORM_MINMAX, CV_32FC1);
  cv::normalize(intMap, denormalizedChannels[1], 0, maxInt, cv::NORM_MINMAX, CV_32FC1);

  cv::merge(denormalizedChannels, 2, denormRiWithI);
}


void RiConverter::saveRiToFile(cv::Mat ri, std::string fileName, FileFormat format)
{
  std::vector<int> imwriteFlag;
  switch(format)
  {
  case PNG:
    cv::imwrite(fileName, ri);
    break;
  case PPM:
    imwriteFlag.push_back(cv::IMWRITE_PXM_BINARY);
    //imwriteFlag.push_back(9);
    cv::imwrite(fileName, ri, imwriteFlag);
    break;
  }
}

/*
float RiConverter::calcRiQuantError(types::PclPcXyz pc, cv::Mat *ri)
{
  int riElem = cv::countNonZero(*ri);
  int pcElem = pc->size();
  int error = (pcElem > riElem) ? pcElem - riElem : riElem - pcElem;

  types::PclPcXyz reconstructedPC = reconstructPcFromRi(ri, true);
  float samplingError = (float)(pcElem - reconstructedPC->size())/(float)pcElem;
  reconstructedPC->clear();
  return samplingError;
}


void RiConverter::calcRiYawxNormError(cv::Mat *ri, double riMax, cv::Mat *nRi)
{
  cv::Mat dnRi, diff;
  cv::Mat mean, stddev;

  denormalizeRi(nRi, riMax, &dnRi);

  cv::absdiff(*ri, dnRi, diff);
  cv::meanStdDev(diff, mean, stddev);

  printf("========= calcRiYawxNormError =======\n");
  printf("\tDistance absdiff: mean (%fm), stddev (%fm)\n", mean.at<double>(0), stddev.at<double>(0));
  printf("====================================\n");
}
*/
