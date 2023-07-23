#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>

namespace flicr
{
namespace types
{
  using PclXyz    = pcl::PointXYZ;
  using PclXyzi   = pcl::PointXYZI;
  using PclPcXyz  = pcl::PointCloud<PclXyz>::Ptr;
  using PclPcXyzi = pcl::PointCloud<PclXyzi>::Ptr;
  using PclMesh   = pcl::PolygonMeshPtr;


  typedef struct RawPc
  {
    uint32_t numOfPoints;
    float *buf;
  } RawPc;


  static PclPcXyz xyzi2xyz(PclPcXyzi pcXyzi)
  {
    PclPcXyz pcXyz(new pcl::PointCloud<PclXyz>);

    for(int i = 0; i < (int)pcXyzi->size(); i++)
    {
      PclXyz p;
      p.x = pcXyzi->points[i].x;
      p.y = pcXyzi->points[i].y;
      p.z = pcXyzi->points[i].z;

      pcXyz->push_back(p);
    }

    return pcXyz;
  }

  class DownsampledTile
  {
    public:
    float yawPrec, pitchPrec;
    float yawOffset, pitchOffset;

    void set(float _yawPrec, float _pitchPrec, float _yawOffset, float _pitchOffset)
    {
      yawPrec = _yawPrec;
      pitchPrec = _pitchPrec;
      yawOffset = _yawOffset;
      pitchOffset = _pitchOffset;
    }

    void print(int idx = 0)
    {
      std::cout << "DownsampledTile: " << idx << std::endl;
      std::cout << "\t yawPrec: " << yawPrec << std::endl;
      std::cout << "\t pitchPrec: " << pitchPrec << std::endl;
      std::cout << "\t yawOffset: " << yawOffset << std::endl;
      std::cout << "\t pitchOffset: " << pitchOffset << std::endl;
    }
  };
}
}

