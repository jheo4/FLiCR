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

  pc1     = hdl64PcReader.getNextPC();
  pc2     = hdl64PcReader.getNextPC();

  cv::Mat *ri1  = hdl64RiConverter.convertPc2Ri(pc1);

  st = getTsNow();
  cv::Mat *ri2  = hdl64RiConverter.convertPc2Ri(pc2);
  et = getTsNow();
  debug_print("RI Convert %f ms", et-st);

  cv::Mat riDiff = *ri2 - *ri1;
  cv::Mat nRiDiff;
  double riDiffMin, riDiffMax;
  hdl64RiConverter.normalizeRi(&riDiff, &nRiDiff, &riDiffMin, &riDiffMax);
  cout << "Min/Max: " << riDiffMin << ", " << riDiffMax << std::endl;

  cv::Mat nRi1, nRi2;
  double riMax1, riMax2;

  // normalize RIs
  hdl64RiConverter.normalizeRi(ri1, &nRi1, &riMax1);
  st = getTsNow();
  hdl64RiConverter.normalizeRi(ri2, &nRi2, &riMax2);
  et = getTsNow();
  debug_print("Norm %f ms", et-st);

  // get naive diffs
  st = getTsNow();
  cv::Mat ri2subri1 = nRi2 - nRi1;  // points only in ri2
  cv::Mat ri1subri2 = nRi1 - nRi2;  // points only in ri1
  et = getTsNow();
  debug_print("Diff points %f ms", et-st);

  int ps1=0, ps2=0;
  for(int row = 0; row < nRi1.rows; row++)
  {
    for(int col = 0; col < nRi1.cols; col++)
    {
      if(ri1subri2.at<float>(row, col) > 0) ps1++;
      if(ri2subri1.at<float>(row, col) > 0) ps2++;
    }
  }
  debug_print("Diff Ri1->Ri2 %d, (%d + %d)", ps1+ps2, ps1, ps2);


  RunLengthCompressor rlCompressor;
  st = getTsNow();
  std::vector<char> rlEncoded1 = rlCompressor.encode((char*)ri2subri1.data, ri2subri1.elemSize()*ri2subri1.total());
  std::vector<char> rlEncoded2 = rlCompressor.encode((char*)ri1subri2.data, ri1subri2.elemSize()*ri1subri2.total());
  et = getTsNow();
  debug_print("RL encode %f ms, size %d", et-st, rlEncoded1.size() + rlEncoded2.size());

  st = getTsNow();
  std::vector<char> rlDecoded1 = rlCompressor.decode(rlEncoded1, ri2subri1.elemSize()*ri2subri1.total());
  std::vector<char> rlDecoded2 = rlCompressor.decode(rlEncoded2, ri1subri2.elemSize()*ri1subri2.total());
  et = getTsNow();
  debug_print("RL decode %f ms", et-st);

  // deflate diffs
  BoostZip boostZip;
  std::vector<char> compressedRi2P, compressedRi1P;
  st = getTsNow();
  boostZip.deflateGzip((char*)ri2subri1.data, ri2subri1.elemSize()*ri2subri1.total(), compressedRi2P);
  boostZip.deflateGzip((char*)ri1subri2.data, ri1subri2.elemSize()*ri1subri2.total(), compressedRi1P);
  et = getTsNow();
  debug_print("Gzip Deflation %f ms", et-st);
  debug_print("Original Data Size %ld, Compressed Data Size %ld",
              ri1subri2.elemSize()*ri1subri2.total()*2, compressedRi1P.size()+compressedRi2P.size());

  st = getTsNow();
  std::vector<char> rlDiffEncoded = rlCompressor.encode((char*)nRiDiff.data, nRiDiff.elemSize()*nRiDiff.total());
  et = getTsNow();
  debug_print("RL encode rlDiff %f ms, %d", et-st, rlDiffEncoded.size());

  std::vector<char> rlDiffEncoded2;
  st = getTsNow();
  boostZip.deflateGzip((char*)nRiDiff.data, nRiDiff.elemSize()*nRiDiff.total(), rlDiffEncoded2);
  et = getTsNow();
  debug_print("RL encode rlDiff %f ms, %d", et-st, rlDiffEncoded2.size());



  // Gzip test
  std::vector<char> riTest;
  st = getTsNow();
  boostZip.deflateGzip((char*)nRi1.data, nRi1.elemSize()*nRi1.total(), riTest);
  et = getTsNow();
  debug_print("[riTest] %f ms", et-st);
  debug_print("[riTest] Original Data Size %ld, Compressed Data Size %ld",
              nRi1.elemSize()*nRi2.total(), riTest.size());



  // inflate diffs
  std::vector<char> decompressedRi2P, decompressedRi1P;
  st = getTsNow();
  boostZip.inflateGzip(compressedRi2P, decompressedRi2P);
  boostZip.inflateGzip(compressedRi1P, decompressedRi1P);
  et = getTsNow();
  debug_print("Gzip Inflation %f ms", et-st);
  debug_print("Compressed Data Size %ld, Decompressed Data Size %ld",
              compressedRi1P.size()+compressedRi2P.size(), decompressedRi2P.size()+decompressedRi1P.size());


  // recreate Ri2 from Ri1 and diffs
  st = getTsNow();
  cv::Mat test = nRi1 + ri2subri1 - ri1subri2;
  et = getTsNow();
  debug_print("dec exe %f ms", et-st);

  /*
  std::ofstream fout1, fout2;
  fout1.open("ri2subri1.bin", std::ios::out | std::ios::binary);
  fout2.open("ri1subri2.bin", std::ios::out | std::ios::binary);
  if (fout1.is_open()){
    fout1.write((const char*)ri2subri1.data, ri2subri1.elemSize() * ri2subri1.total());
    debug_print("raw size of ri2subri1: %d", ri2subri1.elemSize() * ri2subri1.total());
    fout1.close();
  }

  if (fout2.is_open()){
    fout2.write((const char*)ri1subri2.data, ri1subri2.elemSize() * ri1subri2.total());
    debug_print("raw size of ri1subri2: %d", ri1subri2.elemSize() * ri1subri2.total());
    fout2.close();
  }
  */

  while(1)
  {
    cv::imshow("nRi1", nRi1);
    cv::imshow("nRi2", nRi2);
    cv::imshow("Ri2 - Ri1", ri2subri1);
    cv::imshow("Ri1 - Ri2", ri1subri2);
    cv::imshow("Test", test);
    cv::waitKey(10);
  }


  /*
  for(int i = 0; i < oxtsReader.getNumMsg(); i++)
  {
    OxtsMsg oxtsMsg = oxtsReader.getNextMsg();
    oxtsMsg.print();
    std::cout << std::endl;
  }
  */

  return 0;
}

