#include <bits/stdc++.h>
#include <types.h>
#include <cv.hpp>

#ifndef __PCC_HDL64RICONVERTER__
#define __PCC_HDL64RICONVERTER__

class HDL64RIConverter
{
  double thetaPrecision, piPrecision;
  int riRow, riCol;
  public:
    HDL64RIConverter(double thetaPrecision = 0.42, double piPrecision = 0.18);

    cv::Mat* convertPC2RI(std::vector<HDL64PointCloud> *pc);
    cv::Mat* convertPC2RIwithXYZ(std::vector<HDL64PointCloud> *pc);
};

#endif

