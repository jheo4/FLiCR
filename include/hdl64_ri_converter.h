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
    HDL64RIConverter();

    cv::Mat* convertPC2RI(std::vector<HDL64PointCloud> *pc);
    cv::Mat* convertPC2RIwithXYZ(std::vector<HDL64PointCloud> *pc);
    std::vector<HDL64PointCloud>* convertRI2PC(cv::Mat *ri);

    int getRIConvError(std::vector<HDL64PointCloud> *pc, cv::Mat *ri);
    double getRIQuantError(cv::Mat *ri, cv::Mat *nRi, double max);
    double getE2Error(cv::Mat *ri, cv::Mat *nRi){};

    int normRi(cv::Mat *oRi, cv::Mat *nRi);
    void denormRi(cv::Mat *nRi, int max, cv::Mat *dnRi);
};

#endif

