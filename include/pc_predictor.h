#include <bits/stdc++.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>
#include <defs.h>

#ifndef __PCC_PCPREDICTOR__
#define __PCC_PCPREDICTOR__

class PcPredictor
{
  public:
    PclPcXYZ predictNextPc(PclPcXYZ origPc, VecXyz velocity);
};

#endif

