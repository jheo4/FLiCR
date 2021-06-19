#include <bits/stdc++.h>
#include <types.h>
#include <opencv2/opencv.hpp>

#ifndef __PCC_HDL64RICONVERTER__
#define __PCC_HDL64RICONVERTER__

class HDL64RIConverter
{
  double thetaPrecision, piPrecision;
  double thetaOffset,    piOffset;
  int riRow, riCol;

  public:
    HDL64RIConverter(double thetaPrecision = 0.4187, double piPrecision = 0.08);

    cv::Mat* convertPC2RI(std::vector<HDL64PointCloud> *pc);
    cv::Mat* convertPC2RIwithXYZ(std::vector<HDL64PointCloud> *pc);
    std::vector<HDL64PointCloud>* convertRI2PC(cv::Mat *ri);
};

#endif

