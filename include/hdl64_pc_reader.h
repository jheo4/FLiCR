#include <bits/stdc++.h>
#include <bag_reader.h>
#include <types.h>

#ifndef __PCC_HDL64PCREADER__
#define __PCC_HDL64PCREADER__

class HDL64PCReader: public BagReader
{
  public:
    HDL64PCReader();
    HDL64PCReader(std::string bagFile, std::string topic);
    std::vector<HDL64PointCloud>* getNextPC();
};

#endif

