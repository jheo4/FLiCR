#include <ri_converter.h>
#include <defs.h>

using namespace flicr;

RiConverter::RiConverter(double minRange,          double maxRange,
                         double thetaPrecision,    double piPrecision,
                         double thetaDegree,       double piDegree,
                         double thetaDegreeOffset, double piDegreeOffset)
{
  setConfig(minRange, maxRange, thetaPrecision, piPrecision, thetaDegree, piDegree, thetaDegreeOffset, piDegreeOffset);
}


void RiConverter::setConfig(double minRange,          double maxRange,
                            double thetaPrecision,    double piPrecision,
                            double thetaDegree,       double piDegree,
                            double thetaDegreeOffset, double piDegreeOffset)
{
  this->minRange             = minRange;
  this->maxRange             = maxRange;
  this->thetaPrecision       = thetaPrecision;
  this->piPrecision          = piPrecision;
  this->thetaDegree          = thetaDegree;
  this->piDegree             = piDegree;
  this->thetaDegreeOffset    = thetaDegreeOffset;
  this->piDegreeOffset       = piDegreeOffset;

  this->thetaOffset = thetaDegreeOffset/thetaPrecision;
  this->piOffset    = piDegreeOffset   /piPrecision;

  this->riRow = thetaDegree / thetaPrecision;
  this->riCol = piDegree    / piPrecision;
}


void RiConverter::XYZ2RTP(float &x, float &y, float &z, float &rho, int &thetaRow, int &piCol)
{
  /*
    float rTheta = std::acos(z/rho);
    float rPi    = std::atan2(y, x);

    float theta  = RAD2DEGREE(rTheta);
    float pi     = RAD2DEGREE(rPi);

    float nTheta = theta + thetaDegreeOffset;
    float nPi    = pi + piDegreeOffset;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(nTheta/thetaPrecision)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(nPi/piPrecision)));
  */
  rho    = std::sqrt(x*x + y*y + z*z);
  thetaRow = (int)((RAD2DEGREE(std::acos(z/rho))+thetaDegreeOffset) / thetaPrecision);
  piCol    = (int)((RAD2DEGREE(std::atan2(y, x))+piDegreeOffset)    / piPrecision);
}


void RiConverter::RTP2XYZ(float &rho, int &thetaRow, int &piCol, float &x, float &y, float &z)
{
  /*
  float nTheta = (thetaRow * thetaPrecision);
  float nPi    = (piCol * piPrecision);

  float theta = (thetaRow * thetaPrecision) - thetaDegreeOffset;
  float pi    = (piCol    * piPrecision)    - piDegreeOffset;

  float rTheta = DEGREE2RAD(theta);
  float rPi    = DEGREE2RAD(pi);
  */

  float dTheta = (thetaRow * thetaPrecision) - thetaDegreeOffset;
  float dPi    = (piCol    * piPrecision)    - piDegreeOffset;

  float rTheta = DEGREE2RAD(dTheta);
  float rPi    = DEGREE2RAD(dPi);

  x = rho * std::sin(rTheta) * std::cos(rPi);
  y = rho * std::sin(rTheta) * std::sin(rPi);
  z = rho * std::cos(rTheta);
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
    int thetaRow, piCol;

    XYZ2RTP(x, y, z, rho, thetaRow, piCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, thetaRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, piCol));

    outRi.at<float>(rowIdx, colIdx) = rho;
  }
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
    int thetaRow, piCol;

    XYZ2RTP(x, y, z, rho, thetaRow, piCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, thetaRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, piCol));

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
    int thetaRow, piCol;

    XYZ2RTP(x, y, z, rho, thetaRow, piCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, thetaRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, piCol));

    outRi.at<cv::Vec2f>(rowIdx, colIdx)[0] = rho;
    outRi.at<cv::Vec2f>(rowIdx, colIdx)[1] = inPc->points[i].intensity;
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
    int thetaRow, piCol;

    XYZ2RTP(x, y, z, rho, thetaRow, piCol);

    int rowIdx = std::min(outRi.rows-1, std::max(0, thetaRow));
    int colIdx = std::min(outRi.cols-1, std::max(0, piCol));

    outRi.at<float>(rowIdx, colIdx) = rho;
    outIntMap.at<float>(rowIdx, colIdx) = inPc->points[i].intensity;
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
    int thetaRow, piCol;

    XYZ2RTP(x, y, z, rho, thetaRow, piCol);

    int rowIdx = std::min(outRiWithXyz.rows-1, std::max(0, thetaRow));
    int colIdx = std::min(outRiWithXyz.cols-1, std::max(0, piCol));

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


void RiConverter::calcRiPixNormError(cv::Mat *ri, double riMax, cv::Mat *nRi)
{
  cv::Mat dnRi, diff;
  cv::Mat mean, stddev;

  denormalizeRi(nRi, riMax, &dnRi);

  cv::absdiff(*ri, dnRi, diff);
  cv::meanStdDev(diff, mean, stddev);

  printf("========= calcRiPixNormError =======\n");
  printf("\tDistance absdiff: mean (%fm), stddev (%fm)\n", mean.at<double>(0), stddev.at<double>(0));
  printf("====================================\n");
}
*/
