#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/ply_io.h>

#include <types.h>
#include <defs.h>

namespace flicr
{
class PcReader
{
  public:
    PcReader();
    types::RawPc rawReadXyzBin(std::string fileName);

    types::PclPcXyz  readXyzBin (std::string fileName);
    types::PclPcXyz  readXyzFromXyziBin (std::string fileName);
    types::PclPcXyzi readXyziBin(std::string fileName);

    bool readXyzInt(std::string fileName, types::PclPcXyz &pc, std::vector<float> &intensity);

    types::PclPcXyz  readXyzPcd (std::string fileName);
    types::PclPcXyzi readXyziPcd(std::string fileName);

    types::PclPcXyz  readXyzPly (std::string fileName);
    types::PclPcXyzi readXyziPly(std::string fileName);
    types::PclMesh   readMeshPly(std::string fileName);

    void generateMeshFromXyz (types::PclPcXyz pc, pcl::PolygonMeshPtr &mesh,
                              std::vector<int> partID, std::vector<int> pointStates);
};
}

