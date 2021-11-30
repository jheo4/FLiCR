#include <3dpcc>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

using namespace std;

int main() {
  // for profiling
  double st, et;

  /* Set configs from yaml */
  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config         = YAML::LoadFile(configYaml);
  std::string kittiBagFile  = config["kitti_bag_file"].as<std::string>();
  std::string kittiVelodyne = config["kitti_velodyne_topic"].as<std::string>();

  /* Set classes */
  HDL64PCReader hdl64PcReader(kittiBagFile, kittiVelodyne);
  //pcl::io::compression_Profiles_e xyzProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::compression_Profiles_e xyzProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(xyzProfile);
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  PclPcXYZ pc1, pcReconstructed(new pcl::PointCloud<PclXYZ>());
  pc1 = hdl64PcReader.getNextPC();

  std::stringstream compressedData;

  st = getTsNow();
  xyzEncoder->encodePointCloud(pc1, compressedData);
  et = getTsNow();

  debug_print("encoding time: %f ms", et - st);

  st = getTsNow();
  xyzDecoder->decodePointCloud(compressedData, pcReconstructed);
  et = getTsNow();
  debug_print("decoding time: %f ms", et - st);

  compressedData.seekg(0, ios::end);
  int size = compressedData.tellg();
  debug_print("compressed Size: %d", size);

  float PSNR = calcPSNR(pc1, pcReconstructed, 80);
  debug_print("PCL PSNR: %f", PSNR);

  Visualizer visualizer;
  visualizer.initViewerXYZ();

  /* PC Visualization */
  visualizer.setViewer(pcReconstructed);
  for(int i = 0; i < 1; i++) {
    visualizer.show(5000);
  }

  return 0;
}

