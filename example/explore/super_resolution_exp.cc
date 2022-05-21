#include <flicr>

using namespace std;
using namespace flicr;

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
  visualizer.initViewerXyz();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);
  numScans = 1;

  double piPrec[6] = {HDL64_PI_PRECISION_4500,
                      HDL64_PI_PRECISION_4096,
                      HDL64_PI_PRECISION_2048,
                      HDL64_PI_PRECISION_1024,
                      HDL64_PI_PRECISION_512,
                      HDL64_PI_PRECISION_256};

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = "/home/jin/mnt/Data/kitti/City/2011_09_26_drive_0051_sync/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/0000000000.bin";
    os.str(""); os.clear();

    // read data
    types::PclPcXyz pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

    visualizer.setViewer(pcXyz);
    visualizer.show(1);
    visualizer.saveToFile("original.png");

    for(int prec = 1; prec < 6; prec++)
    {
      int row = 64;
      int origCol = HDL64_HORIZONTAL_DEGREE/piPrec[prec];

      int newCol  = (float)origCol * 2;
      double newPrec = piPrec[prec] / 2;

      RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                              HDL64_THETA_PRECISION, piPrec[prec],
                              HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                              HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

      debug_print("Original Prec (%f) for %dx%d", riConverter.piPrecision, riConverter.riCol, riConverter.riRow);


      cv::Mat ri;
      cv::Mat nRi, upRi;
      cv::Mat origDnRi, dnRi;
      double riMax, riMin;

      types::PclPcXyz origRecXyz, decPcXyz;

      riConverter.convertPc2Ri(pcXyz, ri, true);
      riConverter.normalizeRi(ri, nRi, riMin, riMax);

      // upscale
      // cv::INTER_LINEAR
      // cv::INTER_NEAREST
      // cv::INTER_CUBIC
      // cv::INTER_LANCZOS4
      // cv::INTER_AREA
      cv::resize(nRi, upRi, cv::Size(newCol, row), cv::INTER_LINEAR);

      cv::imshow("subsampled RI", nRi);
      cv::imshow("enlarged RI from subsampled RI", upRi);
      cv::waitKey();

      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, newPrec,
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

      debug_print("New Prec (%f) for %dx%d", riConverter.piPrecision, riConverter.riCol, riConverter.riRow);


      st = getTsNow();
      riConverter.denormalizeRi(upRi, riMin, riMax, dnRi);
      riConverter.denormalizeRi(nRi, riMin, riMax, origDnRi);

      decPcXyz   = riConverter.reconstructPcFromRi(dnRi, true);
      origRecXyz = riConverter.reconstructPcFromRi(origDnRi, true);

      visualizer.setViewer(origRecXyz);
      visualizer.show(1);
      visualizer.saveToFile(to_string(prec)+"_orig.png");

      visualizer.setViewer(decPcXyz);
      visualizer.show(1);
      visualizer.saveToFile(to_string(prec)+".png");

      origDnRi.release();
      dnRi.release();
      nRi.release();
      upRi.release();
      decPcXyz->clear();
      origRecXyz->clear();

      printProgress((float)idx/(float)numScans);
    }
    printProgress(1);

    pcXyz->clear();
    intensity.clear();
  }

  return 0;
}
