#include <flicr>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("FLiCR", "FLiCR");
  options.add_options()
    ("y, yaml", "YAML file", cxxopts::value<std::string>())
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string yamlConfig;
  if(parsedArgs.count("yaml"))
  {
    yamlConfig = parsedArgs["yaml"].as<std::string>();
    std::cout << "YAML Config: " << yamlConfig << std::endl;
  }
  else
  {
    std::cout << "Invalid YAML Config" << std::endl;
    exit(0);
  }

  double st, et;

  YAML::Node config = YAML::LoadFile(yamlConfig);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  std::ostringstream os;
  PcReader pcReader;

  pcl::io::compression_Profiles_e xyzProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(xyzProfile);
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/pcl_metric.log");
  metricLogger->info("\tSamplingError\tPSNR\tCD\tencTime\tdecTime\tcompSize");

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);
  numScans = 100;

  for(int idx = 0; idx < numScans; idx++)
  {
    float encTime, decTime;
    int compSize;

    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    types::PclPcXyz pcXyz, pcReconstructed(new pcl::PointCloud<types::PclXyz>());
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

    float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, pcReconstructed, 80);
    float CD   = Metrics::calcCdBtwPcs(pcXyz, pcReconstructed);
    float SE   = Metrics::calcPoinNumDiffBtwPcs(pcXyz, pcReconstructed);

    metricLogger->info("\t{}\t{}\t{}\t{}\t{}\t{}", SE, PSNR, CD, encTime, decTime, compSize);

    pcXyz->clear();
    pcReconstructed->clear();

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}

