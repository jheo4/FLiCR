#include <bits/stdc++.h>
#include <bag_reader.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <defs.h>

#ifndef __PCC_HDL64PCREADER__
#define __PCC_HDL64PCREADER__

class HDL64PCReader: public BagReader
{
  public:
    HDL64PCReader();
    HDL64PCReader(std::string bagFile, std::string topic);
    PCLPcPtr getNextPC();
    void printPCInfo(PCLPcPtr pc);
};

#endif

