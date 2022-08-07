#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <boost/filesystem.hpp>

#include <types.h>
#include <defs.h>

namespace flicr
{
class PcWriter
{
  public:
    void writeBin(std::string fileName, types::PclPcXyz pc);
    void writeBin(std::string fileName, types::PclPcXyzi pc);
    void writeBin(std::string path, std::string fileName, types::PclPcXyzi pc);

    void writePcd(std::string fileName, types::PclPcXyz pc, bool compress);
    void writePcd(std::string fileName, types::PclPcXyzi pc, bool compress);

    void writePly(std::string fileName, types::PclPcXyz pc);
    void writePly(std::string fileName, types::PclPcXyzi pc);

    void writePlyFromMesh(std::string fileName, pcl::PolygonMeshPtr mesh);
};
}

