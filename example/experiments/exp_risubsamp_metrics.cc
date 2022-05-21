#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char** argv)
{
  double st, et;
  int riRow, riCol;

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

  YAML::Node config = YAML::LoadFile(yamlConfig);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  double riPrecisions[5] = {HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048, HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  numScans = 100;

  for(int i = 0; i < 5; i++)
  {
    riRow = (int)(HDL64_VERTICAL_DEGREE   / HDL64_THETA_PRECISION);
    riCol = (int)(HDL64_HORIZONTAL_DEGREE / riPrecisions[i]);

    std::shared_ptr<spdlog::logger> pc2nri       = spdlog::basic_logger_st("pc2nri_"+to_string(riCol),
                                                   "logs/"+dataCategory+"/"+to_string(riCol)+"/enc_pc2nri.log");
    std::shared_ptr<spdlog::logger> nri2pc       = spdlog::basic_logger_st("nri2pc"+to_string(riCol),
                                                   "logs/"+dataCategory+"/"+to_string(riCol)+"/dec_nri2pc.log");
    std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger"+to_string(riCol),
                                                   "logs/"+dataCategory+"/"+to_string(riCol)+"/metric.log");
    metricLogger->info("\tSamplingError\tPSNR\tCD");

    std::ostringstream os;
    PcReader pcReader;
    RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, riPrecisions[i],
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

    for(int idx = 0; idx < numScans; idx++)
    {
      os << std::setw(10) << std::setfill('0') << idx;
      std::string fn = lidarDataPath + "/" + os.str() + ".bin";
      os.str(""); os.clear();

      types::PclPcXyz pcXyz = pcReader.readXyzFromXyziBin(fn);

      cv::Mat ri;
      cv::Mat nRi;
      double riMax, riMin;

      st = getTsNow();
      riConverter.convertPc2Ri(pcXyz, ri, true);
      riConverter.normalizeRi(ri, nRi, riMin, riMax);
      et = getTsNow();
      pc2nri->info("PC2nRI exe\t{}", et-st);

      cv::Mat decRi;
      types::PclPcXyz decXyz;

      st = getTsNow();
      riConverter.denormalizeRi(nRi, riMin, riMax, decRi);
      decXyz = riConverter.reconstructPcFromRi(decRi, true);
      et = getTsNow();
      nri2pc->info("nRI2PC exe\t{}", et-st);

      // metric logging
      float samplingError = Metrics::calcPoinNumDiffBtwPcs(pcXyz, decXyz);
      float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, decXyz, 80);
      float CD = Metrics::calcCdBtwPcs(pcXyz, decXyz);


      metricLogger->info("\t{}\t{}\t{}", samplingError, PSNR, CD);

      decRi.release();
      decXyz->clear();

      ri.release();
      pcXyz->clear();

      printProgress((float)idx/(float)numScans);
    }
    printProgress(1);
  }

  return 0;
}

