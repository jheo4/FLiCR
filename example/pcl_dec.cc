#include <flicr>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

using namespace std;
using namespace flicr;

// Decode encoded point cloud with PCL's octree compressor
// ./pcl_dec encoded.bin decoded.bin
int main(int argc, char* argv[]) {

  double st, et;

  if(argc != 3) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];

  cout << input << endl;
  cout << output << endl;

  std::ifstream inf;
  std::ofstream outf;

  inf.open(input, std::ios::binary);
  outf.open(output, std::ios::binary);

  PcWriter pcWriter;
  std::stringstream encoded;
  types::PclPcXyz decoded(new pcl::PointCloud<types::PclXyz>());

  inf >> encoded.rdbuf();

  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  st = getTsNow();
  xyzDecoder->decodePointCloud(encoded, decoded);
  et = getTsNow();
  debug_print("exe: %f", et-st);

  pcWriter.writeBin(output, decoded);
  return 0;
}

