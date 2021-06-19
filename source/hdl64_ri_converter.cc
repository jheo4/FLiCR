#include <hdl64_ri_converter.h>
#include <utils.h>
#include <defs.h>

HDL64RIConverter::HDL64RIConverter(double thetaPrecision, double piPrecision):
  thetaPrecision(thetaPrecision), piPrecision(piPrecision)
{
  riRow = HDL64_VERTICAL_DEGREE / thetaPrecision;
  riCol = HDL64_HORIZONTAL_DEGREE / piPrecision;

  thetaOffset = HDL64_VERTICAL_DEGREE_OFFSET / thetaPrecision;
  piOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET / piPrecision;
}

cv::Mat* HDL64RIConverter::convertPC2RIwithXYZ(std::vector<HDL64PointCloud> *pc)
{
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));

  for(auto p: *pc) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    float r = p.r; // need to be encoded?
    p.print();

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

cv::Mat* HDL64RIConverter::convertPC2RI(std::vector<HDL64PointCloud> *pc)
{
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC1, cv::Scalar(0.f, 0.f, 0.f, 0.f));

  for(auto p: *pc) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    //p.print();

    float rho    = std::sqrt(x*x + y*y + z*z);
    float rTheta = std::acos(z/rho);
    float rPi    = std::atan2(y, x);

    float theta  = rTheta * 180.0f/PI;
    float pi     = rPi * 180.0f/PI;

    float nTheta = theta + HDL64_VERTICAL_DEGREE_OFFSET;
    float nPi    = pi + HDL64_HORIZONTAL_DEGREE_OFFSET;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(nTheta/thetaPrecision)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(nPi/piPrecision)));

    if(rowIdx == 30 && colIdx == 2000) {
      debug_print("30/2000: %f %f %f", rho, nTheta, nPi);
      debug_print("       : %f %f %f", x, y, z);
    }
    ri->at<float>(rowIdx, colIdx) = rho;
  }

  return ri;
}

std::vector<HDL64PointCloud>* HDL64RIConverter::convertRI2PC(cv::Mat *ri)
{
  std::vector<HDL64PointCloud> *pc = new std::vector<HDL64PointCloud>;
  int elem = 0;

  for(int y = 0; y <= ri->rows; y++) {
    for(int x = 0; x <= ri->cols; x++) {
      float rho = ri->at<float>(y, x);
      if(rho > 0) {
        float nTheta = (y * thetaPrecision);
        float nPi = (x * piPrecision);

        float theta = nTheta - HDL64_VERTICAL_DEGREE_OFFSET;
        float pi    = nPi    - HDL64_HORIZONTAL_DEGREE_OFFSET;

        float rTheta = theta * PI/180.0f;
        float rPi    = pi    * PI/180.0f;

        HDL64PointCloud p;
        p.x = rho * std::sin(rTheta) * std::cos(rPi);
        p.y = rho * std::sin(rTheta) * std::sin(rPi);
        p.z = rho * std::cos(rTheta);
        pc->push_back(p);

        if(y == 30 && x == 2000) {
          debug_print("         %f %f %f", rho, theta, pi);
          debug_print("       : %f %f %f", p.x, p.y, p.z);
        }
      }
    }
  }

  debug_print("num of PC: %d from %dx%d", elem, ri->rows, ri->cols);

  return pc;
}

