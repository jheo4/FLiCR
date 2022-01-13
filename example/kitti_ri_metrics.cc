#include <3dpcc>

using namespace std;


int main() {
  double st, et;
  int riRow, riCol;

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

  double riPrecisions[5] = {HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048, HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};

  int numScans = 100;

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
    HDL64RIConverter riConverter(HDL64_THETA_PRECISION,
                                 riPrecisions[i],
                                 HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                                 HDL64_HORIZONTAL_DEGREE_OFFSET/riPrecisions[i]);

    for(int idx = 0; idx < numScans; idx++)
    {
      os << std::setw(10) << std::setfill('0') << idx;
      std::string fn = lidarDataPath + "/" + os.str() + ".bin";
      os.str(""); os.clear();

      PclPcXYZ pcXyz = pcReader.readXyzFromXyziBin(fn);

      cv::Mat *ri;
      cv::Mat nRi;
      double riMax, riMin;

      st = getTsNow();
      ri = riConverter.convertPc2Ri(pcXyz);
      riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
      et = getTsNow();
      pc2nri->info("PC2nRI exe\t{}", et-st);

      cv::Mat decRi;
      PclPcXYZ decXyz;

      st = getTsNow();
      riConverter.denormalizeRi(&nRi, riMax, &decRi);
      decXyz = riConverter.reconstructPcFromRi(&decRi);
      et = getTsNow();
      nri2pc->info("nRI2PC exe\t{}", et-st);

      // metric logging
      float samplingError = calcSamplingError(pcXyz, decXyz);
      float PSNR          = calcPSNR(pcXyz, decXyz, 80);
      float CD            = calcCD(pcXyz, decXyz);

      metricLogger->info("\t{}\t{}\t{}", samplingError, PSNR, CD);

      decRi.release();
      decXyz->clear();

      ri->release(); delete ri;
      pcXyz->clear();

      printProgress((float)idx/(float)numScans);
    }
    printProgress(1);
  }

  return 0;
}

