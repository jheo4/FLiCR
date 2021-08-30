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


typedef struct Quaternion
{
  float x;
  float y;
  float z;
  float w;

  Quaternion(): x(0), y(0), z(0), w(0) {}
  std::vector<float> get() { return {x, y, z, w}; }
  void print() { printf("Quaternion: x(%f) y(%f) z(%f) w(%f)\n", x, y, z, w); }
} Quaternion;


typedef struct VecXyz
{
  float x;
  float y;
  float z;

  VecXyz(): x(0), y(0), z(0) {}

  std::vector<float> get() { return {x, y, z}; }

  void print() { printf("x(%f) y(%f) z(%f)\n", x, y, z); }
} VecXyz;


typedef struct OxtsMsg
{
  Quaternion orientation;
  VecXyz angularVelocity;
  VecXyz linearAcceleration;
  VecXyz gpsVelocity;

  std::vector<float> getOrientation() { return orientation.get(); };
  std::vector<float> getAngularVelocity() { return angularVelocity.get(); };
  std::vector<float> getLinearAcceleration() { return linearAcceleration.get(); };
  std::vector<float> getGpsVelocity() { return gpsVelocity.get(); };

  void print()
  {
    orientation.print();

    printf("angularVelocity: ");
    angularVelocity.print();

    printf("linearAcceleration: ");
    linearAcceleration.print();

    printf("gpsVelocity: ");
    gpsVelocity.print();
  }
} OxtsMsg;


#endif

