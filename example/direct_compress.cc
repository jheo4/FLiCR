#include <3dpcc>

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
  std::string kittiImu      = config["kitti_imu_topic"].as<std::string>();
  std::string kittiGps      = config["kitti_gps_topic"].as<std::string>();

  /* Set classes */
  HDL64PCReader hdl64PcReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RiConverter;

  PclPcXYZ pc1, pc2, pPc2;
  OxtsMsg oxtsMsg;

  pc1 = hdl64PcReader.getNextPC();

  st = getTsNow();
  cv::Mat *ri1  = hdl64RiConverter.convertPc2Ri(pc1);
  et = getTsNow();
  debug_print("RI Convert %f ms", et-st);
  hdl64RiConverter.calcRiQuantError(pc1, ri1);

  cv::Mat nRi1;
  double riMax1;

  // normalize RIs
  st = getTsNow();
  hdl64RiConverter.normalizeRi(ri1, &nRi1, &riMax1);
  et = getTsNow();
  debug_print("Norm %f ms", et-st);
  hdl64RiConverter.calcRiPixNormError(ri1, riMax1, &nRi1);

  // deflate nRi1
  BoostZip boostZip;
  std::vector<char> compressednRi1;
  st = getTsNow();
  boostZip.deflateGzip((char*)nRi1.data, nRi1.elemSize()*nRi1.total(), compressednRi1);
  et = getTsNow();
  debug_print("nRi1 Gzip Deflation %f ms", et-st);
  debug_print("Original Data Size %ld, Compressed Data Size %ld", nRi1.elemSize()*nRi1.total(), compressednRi1.size());

  // inflate nRi1
  std::vector<char> decompressednRi1;
  st = getTsNow();
  boostZip.inflateGzip(compressednRi1, decompressednRi1);
  et = getTsNow();
  debug_print("Gzip Inflation %f ms", et-st);
  debug_print("Compressed Data Size %ld, Decompressed Data Size %ld", compressednRi1.size(), decompressednRi1.size());


  RunLengthCompressor rlCompressor;
  st = getTsNow();
  std::vector<char> rlEncoded = rlCompressor.encode((char*)nRi1.data, nRi1.elemSize()*nRi1.total());
  et = getTsNow();
  debug_print("RL encode: %d, %fms", rlEncoded.size(), et-st);
  st = getTsNow();
  std::vector<char> rlDecoded = rlCompressor.decode(rlEncoded, nRi1.elemSize()*nRi1.total());
  et = getTsNow();
  debug_print("RL decode: %d, %fms", rlDecoded.size(), et-st);

  // recreate nRi1
  //cv::Mat test(nRi1.rows, nRi1.cols, CV_8UC1, decompressednRi1.data());
  cv::Mat test(nRi1.rows, nRi1.cols, CV_8UC1, rlDecoded.data());

  cv::Mat reRi1;
  hdl64RiConverter.denormalizeRi(&test, riMax1, &reRi1);
  pc2 = hdl64RiConverter.reconstructPcFromRi(&reRi1);
  hdl64RiConverter.calcE2eDistance(pc1, pc2);

  Visualizer visualizer;
  visualizer.initViewerXYZ();

  cv::Mat reconstructedRi1;
  hdl64RiConverter.denormalizeRi(&test, riMax1, &reconstructedRi1);
  PclPcXYZ pcReconstructed = hdl64RiConverter.reconstructPcFromRi(&reconstructedRi1);

  /* PC Visualization */
  visualizer.setViewer(pcReconstructed);
  for(int i = 0; i < 1; i++) {
    visualizer.show(5000);
  }

  return 0;
}

