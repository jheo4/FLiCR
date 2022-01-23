#include <3dpcc>

using namespace std;

int main() {
  double st, et;

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
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/metric.log");

  metricLogger->info("\tSamplingError\tpcPSNR\tCD");

  std::ostringstream os;
  PcReader pcReader;

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
    while(ent = readdir(dir))
    {
      if(!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, "..")) {}
      else
      {
        numScans++;
      }
    }
  }
  closedir(dir);
  debug_print("# of scans: %d", numScans);
  numScans = 1;

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048, HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};
  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = "/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/0000000000.bin";
    os.str(""); os.clear();

    PclPcXYZ pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

    for(int prec = 0; prec < 6; prec++)
    {
      HDL64RIConverter riConverter(HDL64_THETA_PRECISION, piPrec[prec],
                                   HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                                   HDL64_HORIZONTAL_DEGREE_OFFSET/piPrec[prec]);
      debug_print("%f: %d x %d",  riConverter.piPrecision, riConverter.riCol, riConverter.riRow);

      PclPcXYZ decPcXyz;

      /* pc -> ri -> nRi -> yuv -> encoded bytes */
      cv::Mat *ri;
      cv::Mat nRi, dnRi;
      double riMax, riMin;

      st = getTsNow();
      ri = riConverter.convertPc2Ri(pcXyz); // TODO: need to encode intensity also
      riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
      et = getTsNow();
      pc2nri->info("PC2nRI exe\t{}", et-st);

      st = getTsNow();
      riConverter.denormalizeRi(&nRi, riMax, &dnRi);
      decPcXyz = riConverter.reconstructPcFromRi(&dnRi);
      et = getTsNow();
      nri2pc->info("nRI2PC exe\t{}", et-st);

      // metric logging
      float samplingError = calcSamplingError(pcXyz, decPcXyz);
      float PSNR = calcPSNR(pcXyz, decPcXyz, 80);
      float CD   = calcCD(pcXyz, decPcXyz);

      metricLogger->info("\t{}\t{}\t{}", samplingError, PSNR, CD);

      visualizer.setViewer(decPcXyz);
      visualizer.show(500);
      visualizer.saveToFile(to_string(prec)+".png");

      dnRi.release();
      nRi.release();
      decPcXyz->clear();
      printProgress((float)idx/(float)numScans);
    }
    printProgress(1);

    pcXyz->clear();
    intensity.clear();
  }

  return 0;
}
