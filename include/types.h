#include <bits/stdc++.h>

#ifndef __PCC_TYPES__
#define __PCC_TYPES__

typedef struct HDL64PointCloud
{
  float x;
  float y;
  float z;
  float r;

  HDL64PointCloud(): x(0), y(0), z(0), r(0) {}
  HDL64PointCloud(float x, float y): x(x), y(y), z(0), r(0) {}
  HDL64PointCloud(float x, float y, float z, float r): x(x), y(y), z(z), r(r) {}

  std::vector<float> getXYZ()  { return {x, y, z}; }
  std::vector<float> getXYZR() { return {x, y, z, r}; }

  void print() { printf("HDL64PointCloud: x(%f) y(%f) z(%f) r(%f)\n", x, y, z, r); }
} HDL64PointCloud;

#endif

