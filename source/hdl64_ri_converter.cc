#include <hdl64_ri_converter.h>
#include <cv.hpp>
#include <utils.h>
#include <defs.h>

HDL64RIConverter::HDL64RIConverter(double thetaPrecision, double piPrecision):
  thetaPrecision(thetaPrecision), piPrecision(piPrecision)
{
  riRow = HDL64_VERTICAL_DEGREE / thetaPrecision;
  riCol = HDL64_HORIZONTAL_DEGREE / piPrecision;
}

cv::Mat* HDL64RIConverter::convertPC2RIwithXYZ(std::vector<HDL64PointCloud> *pc)
{
  cv::Mat *ri = new cv::Mat(riRow, riCol, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));

  for(auto p: *pc) {
    float x = p.x;
    float y = p.y;
    float z = p.z;
    float r = p.r; // need to be encoded?

    float rho   = std::sqrt(x*x + y*y + z*z);
    float theta = (std::acos(z/rho) * 180.0f/PI) / thetaPrecision ;
    float pi    = (std::atan2(y, x) * 180.0f/PI) / piPrecision;

    float thetaOffset = HDL64_VERTICAL_DEGREE_OFFSET / thetaPrecision;
    float piOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET / piPrecision;

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

    float rho   = std::sqrt(x*x + y*y + z*z);
    float theta = (std::acos(z/rho) * 180.0f/PI) / thetaPrecision ;
    float pi    = (std::atan2(y, x) * 180.0f/PI) / piPrecision;

    float thetaOffset = HDL64_VERTICAL_DEGREE_OFFSET / thetaPrecision;
    float piOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET / piPrecision;

    int rowIdx = std::min(ri->rows-1, std::max(0, (int)(theta+thetaOffset)));
    int colIdx = std::min(ri->cols-1, std::max(0, (int)(pi+piOffset)));

    ri->at<float>(rowIdx, colIdx) = rho;
  }

  return ri;
}

