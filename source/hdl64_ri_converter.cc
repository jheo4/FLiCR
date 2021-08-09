#include <hdl64_ri_converter.h>
#include <utils.h>
#include <defs.h>


HDL64RIConverter::HDL64RIConverter()
{
  thetaPrecision = HDL64_THETA_PRECISION;
  piPrecision = HDL64_PI_PRECISION;

  riRow = HDL64_VERTICAL_DEGREE / thetaPrecision;
  riCol = HDL64_HORIZONTAL_DEGREE / piPrecision;

  thetaOffset = HDL64_VERTICAL_DEGREE_OFFSET / thetaPrecision;
  piOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET / piPrecision;
}



cv::Mat* HDL64RIConverter::convertPc2Ri(PclPcXYZ pc)
{
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f));
  double xSum(0), ySum(0), zSum(0), rhoSum(0);

  for(auto p: pc->points) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    xSum += x; ySum += y; zSum += z;

    float rho    = std::sqrt(x*x + y*y + z*z);

    float rTheta = std::acos(z/rho);
    float rPi    = std::atan2(y, x);

    float theta  = rTheta * 180.0f/PI;
    float pi     = rPi * 180.0f/PI;

    float nTheta = theta + HDL64_VERTICAL_DEGREE_OFFSET;
    float nPi    = pi + HDL64_HORIZONTAL_DEGREE_OFFSET;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(nTheta/thetaPrecision)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(nPi/piPrecision)));

    ri->at<float>(rowIdx, colIdx) = rho;
    rhoSum += rho;
  }

  debug_print("avg (xyz): %f, %f, %f, rho(%f)", xSum/pc->size(), ySum/pc->size(), zSum/pc->size(), rhoSum/pc->size());

  return ri;
}


cv::Mat* HDL64RIConverter::convertPc2RiWithI(PclPcXYZI pc)
{
   cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC2, cv::Scalar(0.f));
  double xSum(0), ySum(0), zSum(0), rhoSum(0), iSum(0);

  for(auto p: pc->points) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    xSum += x; ySum += y; zSum += z;

    float rho    = std::sqrt(x*x + y*y + z*z);

    // theta & pi in rad
    float rTheta = std::acos(z/rho);
    float rPi    = std::atan2(y, x);

    // theta & pi in deg
    float theta  = rTheta * 180.0f/PI;
    float pi     = rPi * 180.0f/PI;

    // normalized theta & pi in deg
    float nTheta = theta + HDL64_VERTICAL_DEGREE_OFFSET;
    float nPi    = pi + HDL64_HORIZONTAL_DEGREE_OFFSET;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(nTheta/thetaPrecision)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(nPi/piPrecision)));

    ri->at<cv::Vec2f>(rowIdx, colIdx)[0] = rho;
    ri->at<cv::Vec2f>(rowIdx, colIdx)[1] = p.intensity;
    rhoSum += rho;
    iSum += p.intensity;
  }

  debug_print("avg (xyz): %f, %f, %f, rho(%f), intensity(%f)",
              xSum/pc->size(), ySum/pc->size(), zSum/pc->size(), rhoSum/pc->size(), iSum/pc->size());

  return ri;
}


cv::Mat* HDL64RIConverter::convertPc2RiWithXYZ(PclPcXYZ pc)
{
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));

  for(auto p: pc->points) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    //float r = p.r; // need to be encoded?

    float rho   = std::sqrt(x*x + y*y + z*z);
    float theta = (std::acos(z/rho) * 180.0f/PI) / thetaPrecision ;
    float pi    = (std::atan2(y, x) * 180.0f/PI) / piPrecision;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(theta+thetaOffset)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(pi+piOffset)));

    debug_print("pi(%f), theta(%f): %d %d", pi, theta, colIdx, rowIdx);
    ri->at<cv::Vec4f>(rowIdx, colIdx) = cv::Vec4f(rho, x, y, z);
  }
  return ri;
}


PclPcXYZ HDL64RIConverter::reconstructPcFromRi(cv::Mat *ri)
{
  double xSum(0), ySum(0), zSum(0), rhoSum(0);

  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);

  for(int y = 0; y <= ri->rows; y++) {
    for(int x = 0; x <= ri->cols; x++) {
      //float rho = ri->at<float>(y, x);
      float rho = ri->at<float>(y, x);
      if(rho > 2 && rho < 81) { // rho is between 2~81
        float nTheta = (y * thetaPrecision);
        float nPi = (x * piPrecision);

        float theta = nTheta - HDL64_VERTICAL_DEGREE_OFFSET;
        float pi    = nPi    - HDL64_HORIZONTAL_DEGREE_OFFSET;

        float rTheta = theta * PI/180.0f;
        float rPi    = pi    * PI/180.0f;

        PclXYZ p;

        p.x = rho * std::sin(rTheta) * std::cos(rPi);
        p.y = rho * std::sin(rTheta) * std::sin(rPi);
        p.z = rho * std::cos(rTheta);

        xSum += p.x; ySum += p.y; zSum += p.z;
        rhoSum += rho;

        pc->push_back(p);
      }
    }
  }

  debug_print("avg (xyz): %f, %f, %f, rho(%f)", xSum/pc->size(), ySum/pc->size(), zSum/pc->size(), rhoSum/pc->size());

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


PclPcXYZI HDL64RIConverter::reconstructPcFromRiWithI(cv::Mat *ri)
{
  double xSum(0), ySum(0), zSum(0), rhoSum(0);

  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);

  for(int y = 0; y <= ri->rows; y++) {
    for(int x = 0; x <= ri->cols; x++) {
      float rho       = ri->at<cv::Vec2f>(y, x)[0];
      float intensity = ri->at<cv::Vec2f>(y, x)[1];
      if(rho > 2 && rho < 81) { // rho is between 2~81
        float nTheta = (y * thetaPrecision);
        float nPi = (x * piPrecision);

        float theta = nTheta - HDL64_VERTICAL_DEGREE_OFFSET;
        float pi    = nPi    - HDL64_HORIZONTAL_DEGREE_OFFSET;

        float rTheta = theta * PI/180.0f;
        float rPi    = pi    * PI/180.0f;

        PclXYZI p;

        p.x         = rho * std::sin(rTheta) * std::cos(rPi);
        p.y         = rho * std::sin(rTheta) * std::sin(rPi);
        p.z         = rho * std::cos(rTheta);
        p.intensity = intensity;

        xSum += p.x; ySum += p.y; zSum += p.z;
        rhoSum += rho;

        pc->push_back(p);
      }
    }
  }

  debug_print("avg (xyz): %f, %f, %f, rho(%f)", xSum/pc->size(), ySum/pc->size(), zSum/pc->size(), rhoSum/pc->size());

  pc->width = pc->size();
  pc->height = 1;
  return pc;
}


void HDL64RIConverter::saveRiToFile(cv::Mat ri, std::string fileName, FileFormat format)
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


int HDL64RIConverter::getRIConvError(PclPcXYZ pc, cv::Mat *ri)
{
  int riElem = cv::countNonZero(*ri);
  int pcElem = pc->size();
  int error = (pcElem > riElem) ? pcElem - riElem : riElem - pcElem;
  debug_print("pcElem(%d), riElem(%d): error(%d) %f%%", pcElem, riElem, error, (double)error/pcElem*100);

  PclPcXYZ reconstructedPC = reconstructPcFromRi(ri);
  debug_print("pcElem(%d), pcFromRiElem(%ld): error(%ld)", pcElem, reconstructedPC->size(), pcElem - reconstructedPC->size());
  return error;
}


double HDL64RIConverter::getRIQuantError(cv::Mat *ri, double max, cv::Mat *nRi)
{
  cv::Mat dnRi, diff;
  denormalizeRi(nRi, max, &dnRi);

  cv::absdiff(*ri, dnRi, diff);

  cv::Mat mean, stddev;
  cv::meanStdDev(diff, mean, stddev);

  debug_print("rho diff: mean(%f), stddev(%f)", mean.at<double>(0), stddev.at<double>(0));

  PclPcXYZ pc   = reconstructPcFromRi(ri);
  PclPcXYZ dnPc = reconstructPcFromRi(&dnRi);

  debug_print("pc elem(%d) dnPC elem(%d)", pc->size(), dnPc->size());

  pc->clear();
  dnPc->clear();

  return 0;
}


void HDL64RIConverter::normalizeRi(cv::Mat *origRi, cv::Mat *normRi, double *maxRho)
{
  double min;
  cv::Point minP, maxP;
  cv::minMaxLoc(*origRi, &min, maxRho, &minP, &maxP);

  cv::Mat temp;
  cv::normalize(*origRi, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1); // error here...
  cv::cvtColor(temp, *normRi, cv::COLOR_GRAY2RGB);

  debug_print("min/max: %f %f", min, *maxRho);
}


void HDL64RIConverter::normalizeRiWithI(cv::Mat *origRiWithI, cv::Mat *normRiWithI, double *maxRho, double *maxInt)
{
  double minRho, minInt;
  cv::Point minP, maxP;

  cv::Mat riChannels[2];
  cv::split(*origRiWithI, riChannels);
  cv::minMaxLoc(riChannels[0], &minRho, maxRho, &minP, &maxP);
  cv::minMaxLoc(riChannels[1], &minInt, maxInt, &minP, &maxP);
  debug_print("rho/intensity range: %f~%f / %f~%f", minRho, *maxRho, minInt, *maxInt);

  cv::Mat normalizedChannels[2];
  cv::normalize(riChannels[0], normalizedChannels[0], 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::normalize(riChannels[1], normalizedChannels[1], 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::merge(normalizedChannels, 2, *normRiWithI);
}


void HDL64RIConverter::denormalizeRi(cv::Mat *normRi, double maxRho, cv::Mat *denormRi)
{
  cv::Mat temp;
  cv::cvtColor(*normRi, temp, cv::COLOR_RGB2GRAY);
  cv::normalize(temp, *denormRi, 0, maxRho, cv::NORM_MINMAX, CV_32FC1);
}


void HDL64RIConverter::denormalizeRiWithI(cv::Mat *normRiWithI, double maxRho, double maxInt, cv::Mat *denormRiWithI)
{
  cv::Mat normalizedChannels[2], denormalizedChannels[2];
  cv::split(*normRiWithI, normalizedChannels);

  cv::normalize(normalizedChannels[0], denormalizedChannels[0], 0, maxRho, cv::NORM_MINMAX, CV_32FC1);
  cv::normalize(normalizedChannels[1], denormalizedChannels[1], 0, maxInt, cv::NORM_MINMAX, CV_32FC1);

  cv::merge(denormalizedChannels, 2, *denormRiWithI);
}

