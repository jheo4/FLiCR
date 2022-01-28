#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <utils.h>

PclPcXYZ xyzi2xyz(PclPcXYZI pcXyzi)
{
  PclPcXYZ pcXyz(new pcl::PointCloud<PclXYZ>);

  for(int i = 0; i < (int)pcXyzi->size(); i++)
  {
    PclXYZ p;
    p.x = pcXyzi->points[i].x;
    p.y = pcXyzi->points[i].y;
    p.z = pcXyzi->points[i].y;

    pcXyz->push_back(p);
  }

  return pcXyz;
}

