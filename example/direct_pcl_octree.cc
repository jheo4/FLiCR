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
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  std::ostringstream os;
  PcReader pcReader;

  pcl::io::compression_Profiles_e xyzProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(xyzProfile);
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/pcl_metric.log");
  metricLogger->info("\tSamplingError\tPSNR\tCD\tencTime\tdecTime\tcompSize");

  int numScans = 0;
  DIR *dir = opendir(lidarDataPath.c_str());
  if(dir == NULL)
  {
    debug_print("invalide lidarDataPath in config.yaml");
    return 0;
  }
  else
  {
    struct dirent *ent;
    while(ent = readdir(dir))
    {
      if(!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, "..")) {}
      else
      {
        numScans++;
      }
    }
  }
  closedir(dir);
  debug_print("# of scans: %d", numScans);
  numScans = 100;

  for(int idx = 0; idx < numScans; idx++)
  {
    float encTime, decTime;
    int compSize;

    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    PclPcXYZ pcXyz, pcReconstructed(new pcl::PointCloud<PclXYZ>());
    std::vector<float> intensity;
    pcXyz = pcReader.readXyzFromXyziBin(fn);

    std::stringstream compressedData;
    st = getTsNow();
    xyzEncoder->encodePointCloud(pcXyz, compressedData);
    et = getTsNow();
    encTime = et-st;

    st = getTsNow();
    xyzDecoder->decodePointCloud(compressedData, pcReconstructed);
    et = getTsNow();
    decTime = et-st;

    compressedData.seekg(0, ios::end);
    compSize = compressedData.tellg();

    float PSNR = calcPSNR(pcXyz, pcReconstructed, 80);
    float CD   = calcCD(pcXyz, pcReconstructed);
    float SE   = calcSamplingError(pcXyz, pcReconstructed);

    metricLogger->info("\t{}\t{}\t{}\t{}\t{}\t{}", SE, PSNR, CD, encTime, decTime, compSize);

    pcXyz->clear();
    pcReconstructed->clear();

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}

