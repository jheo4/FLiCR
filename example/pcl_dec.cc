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
int main(int argc, char* argv[]) {
  cxxopts::Options options("pcl_dec", "PCL Octree-based Decoder");
  options.add_options()
    ("i, input", "Encoded input file path", cxxopts::value<std::string>())
    ("o, output", "Decoded output file path", cxxopts::value<std::string>())
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

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./pcl_dec.log");

  double st, et;

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

  encoded.seekg(0, ios::end);
  int encodedSize = encoded.tellg();
  encoded.seekg(0);

  pcWriter.writeBin(output, decoded);

  latencyLogger->info("{}\t{}\t{}", et-st, encodedSize, decoded->size()*3*4);
  if(debug) debug_print("decomp %f, encodedSize %d, decodedSize %ld", et-st, encodedSize, decoded->size()*3*4);

  return 0;
}

