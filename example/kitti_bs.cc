#include <3dpcc>

using namespace std;

int main() {
  double st, et;
  int riRow = (int)(HDL64_VERTICAL_DEGREE   / HDL64_THETA_PRECISION);
  int riCol = (int)(HDL64_HORIZONTAL_DEGREE / HDL64_PI_PRECISION);

  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty())
  {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";

  YAML::Node config = YAML::LoadFile(configYaml);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  std::shared_ptr<spdlog::logger> pc2nri       = spdlog::basic_logger_st("pc2nri", "logs/"+dataCategory+"/enc_pc2nri.log");
  std::shared_ptr<spdlog::logger> nri2pc       = spdlog::basic_logger_st("nri2pc", "logs/"+dataCategory+"/dec_nri2pc.log");
  std::shared_ptr<spdlog::logger> encLogger    = spdlog::basic_logger_st("encLogger", "logs/"+dataCategory+"/enc.log");
  std::shared_ptr<spdlog::logger> decLogger    = spdlog::basic_logger_st("decLogger", "logs/"+dataCategory+"/dec.log");
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/metric.log");

  metricLogger->info("\tSampling Error\tpcPSNR\triPSNR\triSSIM\tnriSSIM");


  std::ostringstream os;
  PcReader pcReader;
  HDL64RIConverter riConverter;
  BoostZip boostZip;

  Visualizer visualizer;
  visualizer.initViewerXYZ();

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
    while(ent = readdir(dir)) numScans++;
  }
  closedir(dir);
  debug_print("# of scans: %d", numScans);

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    PclPcXYZ pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) break;

    /* pc -> ri -> nRi -> yuv -> encoded bytes */
    cv::Mat *ri;
    cv::Mat nRi;
    double riMax, riMin;

    st = getTsNow();
    ri = riConverter.convertPc2Ri(pcXyz); // TODO: need to encode intensity also
    riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
    et = getTsNow();
    pc2nri->info("PC2nRI exe\t{}", et-st);


    std::vector<char> compressedNri;
    st = getTsNow();
    boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), compressedNri);
    et = getTsNow();
    encLogger->info("encoding exe / byte size\t{}\t{}", et-st, compressedNri.size());


    std::vector<char> decompressedNri;
    cv::Mat decRi, decNri;
    PclPcXYZ decXyz;

    /* encoded bytes -> dec_yuv -> dec_nRi -> dec_ri -> dec_pc */
    st = getTsNow();
    boostZip.inflateGzip(compressedNri, decompressedNri);
    decNri = cv::Mat(nRi.rows, nRi.cols, CV_8UC1, decompressedNri.data());
    et = getTsNow();
    decLogger->info("decoding exe\t{}", et-st);

    st = getTsNow();
    riConverter.denormalizeRi(&decNri, riMax, &decRi);
    decXyz = riConverter.reconstructPcFromRi(&decRi);
    et = getTsNow();
    nri2pc->info("nRI2PC exe\t{}", et-st);


    // metric logging
    float samplingError = riConverter.calcRiQuantError(pcXyz, ri);
    float PSNR = calcPSNR(pcXyz, decXyz, riMax);
    float riPSNR = getImgPSNR(*ri, decRi, riMax);
    float riMSSIM1 = getImgMSSIM(nRi, decNri);
    float riMSSIM2 = getImgMSSIM(*ri, decRi);

    metricLogger->info("\t{}\t{}\t{}\t{}\t{}", samplingError, PSNR, riPSNR, riMSSIM1, riMSSIM2);

    visualizer.setViewer(decXyz);
    visualizer.show(1);

    decRi.release();
    decompressedNri.clear();
    decXyz->clear();

    ri->release(); delete ri;
    pcXyz->clear();
    intensity.clear();


    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}
