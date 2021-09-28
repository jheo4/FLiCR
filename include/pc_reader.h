#include <bits/stdc++.h>
#include <bag_reader.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <defs.h>

#ifndef __PCC_PCREADER__
#define __PCC_PCREADER__

class PcReader
{
  public:
    PcReader();
    PclPcXYZ  readXyzBin (std::string fileName);
    PclPcXYZI readXyziBin(std::string fileName);

    PclPcXYZ  readXyzPcd (std::string fileName);
    PclPcXYZI readXyziPcd(std::string fileName);

    PclPcXYZ  readXyzPly (std::string fileName);
    PclPcXYZI readXyziPly(std::string fileName);
};

#endif

