#include <flicr>

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

  std::shared_ptr<spdlog::logger> pc2nri       = spdlog::basic_logger_st("pc2nri", "logs/"+dataCategory+"/enc_pc2nri.log");
  std::shared_ptr<spdlog::logger> nri2pc       = spdlog::basic_logger_st("nri2pc", "logs/"+dataCategory+"/dec_nri2pc.log");
  std::shared_ptr<spdlog::logger> encLogger    = spdlog::basic_logger_st("encLogger", "logs/"+dataCategory+"/enc.log");
  std::shared_ptr<spdlog::logger> decLogger    = spdlog::basic_logger_st("decLogger", "logs/"+dataCategory+"/dec.log");
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/metric.log");

  metricLogger->info("\tSamplingError\tpcPSNR\triPSNR\triSSIM\tnriSSIM");

  std::ostringstream os;
  PcReader pcReader;
  RiConverter riConverter;
  BoostZip boostZip;

  Visualizer visualizer;
  visualizer.initViewerXyz();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    types::PclPcXyz pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) break;

    /* pc -> ri -> nRi -> yuv -> encoded bytes */
    cv::Mat ri;
    cv::Mat nRi;
    double riMax, riMin;

    st = getTsNow();
    riConverter.convertPc2Ri(pcXyz, ri, true); // TODO: need to encode intensity also
    riConverter.normalizeRi(ri, nRi, riMin, riMax);
    et = getTsNow();
    pc2nri->info("PC2nRI exe\t{}", et-st);


    std::vector<char> compressedNri;
    st = getTsNow();
    boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), compressedNri);
    et = getTsNow();
    encLogger->info("encoding exe / byte size\t{}\t{}", et-st, compressedNri.size());


    std::vector<char> decompressedNri;
    cv::Mat decRi, decNri;
    types::PclPcXyz decXyz;

    /* encoded bytes -> dec_yuv -> dec_nRi -> dec_ri -> dec_pc */
    st = getTsNow();
    boostZip.inflateGzip(compressedNri, decompressedNri);
    decNri = cv::Mat(nRi.rows, nRi.cols, CV_8UC1, decompressedNri.data());
    et = getTsNow();
    decLogger->info("decoding exe\t{}", et-st);

    st = getTsNow();
    riConverter.denormalizeRi(decNri, riMin, riMax, decRi);
    decXyz = riConverter.reconstructPcFromRi(decRi, true);
    et = getTsNow();
    nri2pc->info("nRI2PC exe\t{}", et-st);


    // metric logging
    float samplingError = Metrics::calcPoinNumDiffBtwPcs(pcXyz, decXyz);
    float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, decXyz, riMax);
    float riPSNR = Metrics::getImgPSNR(ri, decRi, riMax);
    float riMSSIM1 = Metrics::getImgMSSIM(nRi, decNri);
    float riMSSIM2 = Metrics::getImgMSSIM(ri, decRi);

    metricLogger->info("\t{}\t{}\t{}\t{}\t{}", samplingError, PSNR, riPSNR, riMSSIM1, riMSSIM2);

    visualizer.setViewer(decXyz);
    visualizer.show(1);

    decRi.release();
    decompressedNri.clear();
    decXyz->clear();

    ri.release();
    pcXyz->clear();
    intensity.clear();

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}
