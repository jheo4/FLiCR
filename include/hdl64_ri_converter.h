#include <bits/stdc++.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <defs.h>

#ifndef __PCC_HDL64RICONVERTER__
#define __PCC_HDL64RICONVERTER__

class HDL64RIConverter
{
  public:
    int riRow, riCol;
    double thetaPrecision, piPrecision;
    double thetaOffset,    piOffset;

    HDL64RIConverter(double thetaPrecision = HDL64_THETA_PRECISION,
                     double piPrecision = HDL64_PI_PRECISION,
                     double thetaOffset = HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                     double piOffset    = HDL64_HORIZONTAL_DEGREE_OFFSET/HDL64_PI_PRECISION);

    cv::Mat*  convertRawPc2Ri          (RawPc pc);
    cv::Mat*  convertPc2Ri             (PclPcXYZ pc);
    cv::Mat*  convertPc2RinonP         (PclPcXYZ pc);
    cv::Mat*  convertPc2RiWithI        (PclPcXYZI);
    cv::Mat*  convertPc2RiWithInonP    (PclPcXYZI);
    bool      convertPc2RiWithIm       (PclPcXYZI pc, cv::Mat &ri, cv::Mat &intMap);

    cv::Mat*  convertPc2RiWithXYZ      (PclPcXYZ pc);

    PclPcXYZ  reconstructPcFromRi          (cv::Mat *ri);
    PclPcXYZ  reconstructPcFromRinonP      (cv::Mat *ri);
    PclPcXYZI reconstructPcFromRiWithI     (cv::Mat *ri);
    PclPcXYZI reconstructPcFromRiWithInonP (cv::Mat *ri);
    PclPcXYZI reconstructPcFromRiWithIm    (cv::Mat &ri, cv::Mat &intMap);

    float calcRiQuantError(PclPcXYZ pc, cv::Mat *ri);
    void calcRiPixNormError(cv::Mat *ri, double riMax, cv::Mat *nRi);

    float calcNearestDistance(const pcl::search::KdTree<pcl::PointXYZ> &tree, const pcl::PointXYZ &pt);
    float calcPcAvgDistance(PclPcXYZ pc1, PclPcXYZ pc2, float thresh);

    void calcE2eDistance(PclPcXYZ pc1, PclPcXYZ pc2, float thresh = std::numeric_limits<float>::max());
    void calcE2eDistance(PclPcXYZ pc, double riMax, cv::Mat *nRi);

    void normalizeRi(cv::Mat *origRi, cv::Mat *normRi, double *maxRho);
    void normalizeRi(cv::Mat *origRi, cv::Mat *normRi, double *minRho, double *maxRho);
    void normalizeRiWithI(cv::Mat *origRiWithI, cv::Mat *normRiWithI, double *maxRho, double *maxInt);
    void denormalizeRi(cv::Mat *normRi, double maxRho, cv::Mat *denormRi);
    void denormalizeRiWithI(cv::Mat *normRiWithI, double maxRho, double maxInt, cv::Mat *denormRiWithI);

    enum FileFormat {PNG, PPM};
    void saveRiToFile(cv::Mat ri, std::string fileName, FileFormat format = PNG);
};

#endif

