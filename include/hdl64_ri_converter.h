#include <bits/stdc++.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>
#include <defs.h>

#ifndef __PCC_HDL64RICONVERTER__
#define __PCC_HDL64RICONVERTER__

class HDL64RIConverter
{
  double thetaPrecision, piPrecision;
  double thetaOffset,    piOffset;
  int riRow, riCol;

  public:
    HDL64RIConverter();
    cv::Mat* convertPC2RI(PCLPcPtr pc);
    cv::Mat* convertPC2RIwithXYZ(PCLPcPtr pc);
    PCLPcPtr convertRI2PC(cv::Mat *ri);

    int getRIConvError(PCLPcPtr pc, cv::Mat *ri);
    double getRIQuantError(cv::Mat *ri, cv::Mat *nRi, double max);
    double getE2Error(cv::Mat *ri, cv::Mat *nRi){};

    int normRi(cv::Mat *oRi, cv::Mat *nRi);
    void denormRi(cv::Mat *nRi, int max, cv::Mat *dnRi);
};

#endif

