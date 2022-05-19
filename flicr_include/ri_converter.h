#pragma once

#include <bits/stdc++.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <types.h>

namespace flicr
{
class RiConverter
{
  public:
    int riRow, riCol;
    double thetaPrecision,    piPrecision;
    double thetaDegree,       piDegree;
    double thetaDegreeOffset, piDegreeOffset;
    double thetaOffset,       piOffset;
    double minRange, maxRange;

    RiConverter(double minRange          = HDL64_MIN_RANGE,
                double maxRange          = HDL64_MAX_RANGE,
                double thetaPrecision    = HDL64_THETA_PRECISION,
                double piPrecision       = HDL64_PI_PRECISION,
                double thetaDegree       = HDL64_VERTICAL_DEGREE,
                double piDegree          = HDL64_HORIZONTAL_DEGREE,
                double thetaDegreeOffset = HDL64_VERTICAL_DEGREE_OFFSET,
                double piDegreeOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET);

    void setConfig(double minRange,          double maxRange,
                   double thetaPrecision,    double piPrecision,
                   double thetaDegree,       double piDegree,
                   double thetaDegreeOffset, double piDegreeOffset);

    void XYZ2RTP(float &x, float &y, float &z, float &rho, int &thetaRow, int &piCol);
    void RTP2XYZ(float &rho, int &thetaRow, int &piCol, float &x, float &y, float &z);

    cv::Mat*  convertRawPc2Ri          (types::RawPc pc,     bool parallel);
    cv::Mat*  convertPc2Ri             (types::PclPcXyz pc,  bool parallel);
    cv::Mat*  convertPc2RiWithI        (types::PclPcXyzi,    bool parallel);
    bool      convertPc2RiWithIm       (types::PclPcXyzi pc, cv::Mat &ri, cv::Mat &intMap, bool parallel);
    cv::Mat*  convertPc2RiWithXYZ      (types::PclPcXyz pc,  bool parallel);

    types::PclPcXyz  reconstructPcFromRi       (cv::Mat *ri, bool parallel);
    types::PclPcXyzi reconstructPcFromRiWithI  (cv::Mat *ri, bool parallel);
    types::PclPcXyzi reconstructPcFromRiWithIm (cv::Mat &ri, cv::Mat &intMap, bool parallel);

    void normalizeRi(cv::Mat &origRi, cv::Mat &normRi);
    void normalizeRi(cv::Mat &origRi, cv::Mat &normRi, double *maxRho);
    void normalizeRi(cv::Mat &origRi, cv::Mat &normRi, double *minRho, double *maxRho);
    void normalizeRiWithI(cv::Mat &origRiWithI, cv::Mat &normRiWithI, double *maxRho, double *maxInt);
    void normalizeRiWithI(cv::Mat &origRiWithI, cv::Mat &normRi, cv::Mat &intMap, double *maxRho, double *maxInt);

    void denormalizeRi(cv::Mat &normRi, double minRho, double maxRho, cv::Mat &denormRi);
    void denormalizeRiWithI(cv::Mat &normRiWithI, double maxRho, double maxInt, cv::Mat &denormRiWithI);
    void denormalizeRiWithI(cv::Mat &normRi, cv::Mat &intMap, double maxRho, double maxInt, cv::Mat &denormRiWithI);

    enum FileFormat {PNG, PPM};
    void saveRiToFile(cv::Mat ri, std::string fileName, FileFormat format = PNG);
};
}

