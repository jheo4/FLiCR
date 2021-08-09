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
    cv::Mat*  convertPc2Ri             (PclPcXYZ pc);
    cv::Mat*  convertPc2RiWithI        (PclPcXYZI);
    cv::Mat*  convertPc2RiWithXYZ      (PclPcXYZ pc);
    PclPcXYZ  reconstructPcFromRi      (cv::Mat *ri);
    PclPcXYZI reconstructPcFromRiWithI (cv::Mat *ri);

    int getRIConvError(PclPcXYZ pc, cv::Mat *ri);
    double getRIQuantError(cv::Mat *ri, double max, cv::Mat *nRi);
    double getE2Error(cv::Mat *ri, cv::Mat *nRi){};

    void normalizeRi(cv::Mat *origRi, cv::Mat *normRi, double *maxRho);
    void normalizeRiWithI(cv::Mat *origRiWithI, cv::Mat *normRiWithI, double *maxRho, double *maxInt);
    void denormalizeRi(cv::Mat *normRi, double maxRho, cv::Mat *denormRi);
    void denormalizeRiWithI(cv::Mat *normRiWithI, double maxRho, double maxInt, cv::Mat *denormRiWithI);

    enum FileFormat {PNG, PPM};
    void saveRiToFile(cv::Mat ri, std::string fileName, FileFormat format = PNG);
};

#endif

