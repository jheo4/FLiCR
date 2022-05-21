#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char** argv) {
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

  std::shared_ptr<spdlog::logger> pc2ri = spdlog::basic_logger_st("pc2ri", "logs/enc/"+dataCategory+"/pc2ri.log");
  std::shared_ptr<spdlog::logger> ri2nri = spdlog::basic_logger_st("ri2nri", "logs/enc/"+dataCategory+"/ri2nri.log");

  std::shared_ptr<spdlog::logger> ri2dict = spdlog::basic_logger_st("ri2dict", "logs/enc/"+dataCategory+"/ri2dict.log");
  std::shared_ptr<spdlog::logger> nri2dict = spdlog::basic_logger_st("nri2dict", "logs/enc/"+dataCategory+"/nri2dict.log");

  std::shared_ptr<spdlog::logger> ridict2ri = spdlog::basic_logger_st("ridict2ri", "logs/dec/"+dataCategory+"/ridict2ri.log");
  std::shared_ptr<spdlog::logger> nridict2nri = spdlog::basic_logger_st("nridict2nri", "logs/dec/"+dataCategory+"/nridict2nri.log");

  std::shared_ptr<spdlog::logger> nri2ri = spdlog::basic_logger_st("nri2ri", "logs/dec/"+dataCategory+"/nri2ri.log");
  std::shared_ptr<spdlog::logger> ri2pc = spdlog::basic_logger_st("ri2pc", "logs/dec/"+dataCategory+"/ri2pc.log");

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/dec/"+dataCategory+"/metric.log");
  std::shared_ptr<spdlog::logger> metricLogger2 = spdlog::basic_logger_st("metLogger2", "logs/dec/"+dataCategory+"/metric_nri.log");

  metricLogger->info("RI_Dict\tPSNR\tCD");
  metricLogger2->info("NRI_Dict\tPSNR\tCD");

  std::ostringstream os;
  PcReader pcReader;

  Visualizer visualizer;
  visualizer.initViewerXyz();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);
  numScans = 50;

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048,
                      HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};


  for(int prec = 0; prec < 6; prec++)
  {
    RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, piPrec[prec],
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
    debug_print("%d x %d", riConverter.riCol, riConverter.riRow);

    for(int idx = 0; idx < numScans; idx++)
    {
      os << std::setw(10) << std::setfill('0') << idx;
      std::string fn = lidarDataPath + "/" + os.str() + ".bin";
      debug_print("%s", fn.c_str());
      os.str(""); os.clear();

      types::PclPcXyz pcXyz;
      std::vector<float> intensity;

      if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

      cv::Mat ri;
      cv::Mat nRi, dnRi;
      double riMax, riMin;

      st = getTsNow();
      riConverter.convertPc2Ri(pcXyz, ri, true);
      et = getTsNow();
      pc2ri->info("PC2RI exe\t{}", et-st);

      st = getTsNow();
      riConverter.normalizeRi(ri, nRi, riMax);
      et = getTsNow();
      ri2nri->info("RI2NRI exe\t{}", et-st);

      BoostZip boostZip;
      std::vector<char> compressedRi, compressednRi;
      std::vector<char> decompressedRi, decompressednRi;

      // dict Ri
      st = getTsNow();
      boostZip.deflateGzip((char*)ri.data, ri.elemSize()*ri.total(), compressedRi);
      et = getTsNow();
      ri2dict->info("RI2DICT exe, size\t{}\t{}", et-st, compressedRi.size());

      // dict nRi
      st = getTsNow();
      boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), compressednRi);
      et = getTsNow();
      nri2dict->info("NRI2DICT exe, size\t{}\t{}", et-st, compressednRi.size());


      // ridict Ri
      st = getTsNow();
      boostZip.inflateGzip(compressedRi, decompressedRi);
      et = getTsNow();
      ridict2ri->info("RIDICT2RI exe\t{}", et-st);

      // nridict nRi
      st = getTsNow();
      boostZip.inflateGzip(compressednRi, decompressednRi);
      et = getTsNow();
      nridict2nri->info("NRIDICT2NRI exe\t{}", et-st);

      cv::Mat recRi(ri.rows, ri.cols, CV_32FC1, decompressedRi.data());
      cv::Mat recNri(nRi.rows, nRi.cols, CV_8UC1, decompressednRi.data());
      cv::Mat recRiFromNri;

      st = getTsNow();
      riConverter.denormalizeRi(recNri, 0, riMax, recRiFromNri);
      et = getTsNow();
      nri2ri->info("NRI2RI exe\t{}", et-st);

      types::PclPcXyz recPc1, recPc2;

      st = getTsNow();
      recPc1 = riConverter.reconstructPcFromRi(recRi, true);
      et = getTsNow();
      ri2pc->info("RI2PC exe\t{}", et-st);

      st = getTsNow();
      recPc2 = riConverter.reconstructPcFromRi(recRiFromNri, true);
      et = getTsNow();
      ri2pc->info("RI2PC exe\t{}", et-st);

      float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, recPc1, HDL64_MAX_RANGE);
      float CD   = Metrics::calcCdBtwPcs(pcXyz, recPc1);

      float PSNR2= Metrics::calcPsnrBtwPcs(pcXyz, recPc2, HDL64_MAX_RANGE);
      float CD2  = Metrics::calcCdBtwPcs(pcXyz, recPc2);

      metricLogger->info("\t\t{}\t{}", PSNR, CD);
      metricLogger2->info("\t\t{}\t{}", PSNR2, CD2);

      visualizer.setViewer(recPc1);
      visualizer.show(10);
      visualizer.setViewer(recPc2);
      visualizer.show(10);

      dnRi.release();
      nRi.release();

      recPc1->clear();
      recPc2->clear();

      ri.release();
      nRi.release();
      recRi.release();
      recNri.release();
      recRiFromNri.release();

      printProgress((float)idx/(float)numScans);

      pcXyz->clear();
      intensity.clear();
    }
    printProgress(1);

    os.flush();
    os.clear();
  }

  return 0;
}
