#include <flicr>
#include <bits/stdc++.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

using namespace std;
using namespace flicr;

// Encode raw point cloud with PCL's octree compressor
int main(int argc, char* argv[]) {
  cxxopts::Options options("pcl_enc", "PCL Octree-based Encoder");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("o, output", "Encoded output file path", cxxopts::value<std::string>())
    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string input  = parsedArgs["input"].as<std::string>();
  std::string output = parsedArgs["output"].as<std::string>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput file: " << input << endl;
    cout << "\toutput file: " << output << endl;
  }

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./pcl_enc.log");

  double st, et;

  std::ofstream outf;
  outf.open(output, std::ios::binary);

  PcReader pcReader;
  types::PclPcXyz pcXyz = pcReader.readXyzFromXyziBin(input);
  int rawSize = pcXyz->size() * 3 * 4;

  pcl::io::compression_Profiles_e xyzProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *xyzEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(xyzProfile);

  std::stringstream encoded;

  st = getTsNow();
  xyzEncoder->encodePointCloud(pcXyz, encoded);
  et = getTsNow();

  encoded.seekg(0, ios::end);
  int encodedSize = encoded.tellg();
  encoded.seekg(0);

  latencyLogger->info("{}\t{}\t{}", et-st, rawSize, encodedSize);
  if(debug) debug_print("comp %f, orig size %d, comp size %d", et-st, rawSize, encodedSize);

  outf << encoded.rdbuf();

  return 0;
}

