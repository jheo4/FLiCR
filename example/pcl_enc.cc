#include <3dpcc>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

using namespace std;

int main(int argc, char* argv[]) {
  // ./pcl_enc orig.bin encoded.bin
  double st, et;

  if(argc != 3) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];

  cout << input << endl;
  cout << output << endl;

  std::ofstream outf;
  outf.open(output, std::ios::binary);

  PcReader pcReader;
  PclPcXYZ pcXyz = pcReader.readXyzFromXyziBin(input);

  pcl::io::compression_Profiles_e xyzProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(xyzProfile);

  std::stringstream encoded;
  xyzEncoder->encodePointCloud(pcXyz, encoded);

  outf << encoded.rdbuf();

  return 0;
}

