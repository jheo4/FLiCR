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


cv::Mat* HDL64RIConverter::convertPC2RIwithXYZ(PCLPcPtr pc)
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


cv::Mat* HDL64RIConverter::convertPC2RI(PCLPcPtr pc)
{
  //cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f, 0.f, 0.f, 0.f));
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32SC1, cv::Scalar(0));

  for(auto p: pc->points) {
    float x = p.x;
    float y = p.y;
    float z = p.z;

    float rho    = std::sqrt(x*x + y*y + z*z);
    int nRho     = (int)(rho * 100);

    float rTheta = std::acos(z/rho);
    float rPi    = std::atan2(y, x);

    float theta  = rTheta * 180.0f/PI;
    float pi     = rPi * 180.0f/PI;

    float nTheta = theta + HDL64_VERTICAL_DEGREE_OFFSET;
    float nPi    = pi + HDL64_HORIZONTAL_DEGREE_OFFSET;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(nTheta/thetaPrecision)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(nPi/piPrecision)));

    ri->at<int>(rowIdx, colIdx) = nRho;
  }

  return ri;
}


PCLPcPtr HDL64RIConverter::convertRI2PC(cv::Mat *ri)
{
  PCLPcPtr pc(new pcl::PointCloud<PCLPtXYZ>);

  for(int y = 0; y <= ri->rows; y++) {
    for(int x = 0; x <= ri->cols; x++) {
      //float rho = ri->at<float>(y, x);
      int rho = ri->at<int>(y, x);
      if(rho > 0) {
        float nTheta = (y * thetaPrecision);
        float nPi = (x * piPrecision);

        float theta = nTheta - HDL64_VERTICAL_DEGREE_OFFSET;
        float pi    = nPi    - HDL64_HORIZONTAL_DEGREE_OFFSET;

        float rTheta = theta * PI/180.0f;
        float rPi    = pi    * PI/180.0f;

        PCLPtXYZ p;

        float fRho = (float)rho/100;
        p.x = fRho * std::sin(rTheta) * std::cos(rPi);
        p.y = fRho * std::sin(rTheta) * std::sin(rPi);
        p.z = fRho * std::cos(rTheta);
        pc->push_back(p);

        /*
        if(y == 30 && x == 2000) {
          debug_print("         %f %f %f", rho, nTheta, nPi);
          debug_print("       : %f %f %f", p.x, p.y, p.z);
        }
        */
      }
    }
  }
  return pc;
}


int HDL64RIConverter::getRIConvError(PCLPcPtr pc, cv::Mat *ri)
{
  int riElem = cv::countNonZero(*ri);
  int pcElem = pc->size();
  int error = (pcElem > riElem) ? pcElem - riElem : riElem - pcElem;
  debug_print("pcElem(%d), riElem(%d): error(%d) %f%%", pcElem, riElem, error, (double)error/pcElem*100);

  PCLPcPtr reconstructedPC = convertRI2PC(ri);
  debug_print("pcElem(%d), pcFromRiElem(%ld): error(%ld)", pcElem, reconstructedPC->size(), pcElem - reconstructedPC->size());
  return error;
}


double HDL64RIConverter::getRIQuantError(cv::Mat *ri, cv::Mat *nRi, double max)
{
  cv::Mat dnRi, diff;
  denormRi(nRi, max, &dnRi);

  cv::absdiff(*ri, dnRi, diff);

  cv::Mat mean, stddev;
  cv::meanStdDev(diff, mean, stddev);

  debug_print("rho diff: mean(%f), stddev(%f)", mean.at<double>(0), stddev.at<double>(0));

  PCLPcPtr pc  = convertRI2PC(ri);
  PCLPcPtr dnPc = convertRI2PC(&dnRi);

  debug_print("pc elem(%d) dnPC elem(%d)", pc->size(), dnPc->size());

  pc->clear();
  dnPc->clear();

  return 0;
}


int HDL64RIConverter::normRi(cv::Mat *oRi, cv::Mat *nRi)
{
  double min, max;
  cv::Point minP, maxP;
  cv::normalize(*oRi, *nRi, 0, 255, cv::NORM_MINMAX, CV_8UC1); // error here...
  cv::minMaxLoc(*oRi, &min, &max, &minP, &maxP);

  debug_print("min/max: %f %f", min, max);

  return (int)max;
}


void HDL64RIConverter::denormRi(cv::Mat *nRi, int max, cv::Mat *dnRi)
{
  cv::normalize(*nRi, *dnRi, 0, max, cv::NORM_MINMAX, CV_32SC1);
  //*dnRi *= max;
}

