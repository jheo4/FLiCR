#include <bits/stdc++.h>
#include <bag_reader.h>
#include <types.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <defs.h>

#ifndef __PCC_PCWRITER__
#define __PCC_PCWRITER__

class PcWriter
{
  public:
    PcWriter();
    void writeBin(std::string fileName, PclPcXYZ pc);
    void writeBin(std::string fileName, PclPcXYZI pc);

    void writePcd(std::string fileName, PclPcXYZ pc, bool compress);
    void writePcd(std::string fileName, PclPcXYZI pc, bool compress);

    void writePly(std::string fileName, PclPcXYZ pc);
    void writePly(std::string fileName, PclPcXYZI pc);

    void writePlyFromMesh(std::string fileName, pcl::PolygonMeshPtr mesh);
};

#endif

