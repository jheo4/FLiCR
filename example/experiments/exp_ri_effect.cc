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
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/metric.log");

  metricLogger->info("\tSamplingError\tpcPSNR\tCD");

  std::ostringstream os;
  PcReader pcReader;

  Visualizer visualizer;
  visualizer.initViewerXyz();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);
  numScans = 1;

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048, HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = "/home/jin/mnt/PCC_E2E/3D_OBJ_DET/data/kitti/training/velodyne/000000.bin";
    os.str(""); os.clear();

    types::PclPcXyz pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

    for(int prec = 0; prec < 6; prec++)
    {
      RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                              HDL64_THETA_PRECISION, piPrec[prec],
                              HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                              HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
      debug_print("%f: %d x %d",  riConverter.piPrecision, riConverter.riCol, riConverter.riRow);

      types::PclPcXyz decPcXyz;

      /* pc -> ri -> nRi -> yuv -> encoded bytes */
      cv::Mat ri;
      double riMax, riMin;

      st = getTsNow();
      riConverter.convertPc2Ri(pcXyz, ri, true);
      et = getTsNow();
      pc2nri->info("PC2nRI exe\t{}", et-st);

      st = getTsNow();
      decPcXyz = riConverter.reconstructPcFromRi(ri, true);
      et = getTsNow();
      nri2pc->info("nRI2PC exe\t{}", et-st);

      // metric logging
      float samplingError = Metrics::calcPoinNumDiffBtwPcs(pcXyz, decPcXyz);
      float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, decPcXyz, 80);
      float CD   = Metrics::calcCdBtwPcs(pcXyz, decPcXyz);

      metricLogger->info("\t{}\t{}\t{}", samplingError, PSNR, CD);

      visualizer.setViewer(decPcXyz);
      visualizer.show(1);
      visualizer.saveToFile(to_string(prec)+".png");

      decPcXyz->clear();
      printProgress((float)idx/(float)numScans);
    }
    printProgress(1);

    pcXyz->clear();
    intensity.clear();
  }

  return 0;
}
