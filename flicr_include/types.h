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
      p.z = pcXyzi->points[i].y;

      pcXyz->push_back(p);
    }

    return pcXyz;
  }
}
}

