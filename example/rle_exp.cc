#include "opencv2/core/hal/interface.h"
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
  numScans = 50;

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048,
                      HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};


  for(int prec = 0; prec < 6; prec++)
  {
    HDL64RIConverter riConverter(HDL64_THETA_PRECISION, piPrec[prec],
        HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
        HDL64_HORIZONTAL_DEGREE_OFFSET/piPrec[prec]);
    debug_print("%d x %d", riConverter.riCol, riConverter.riRow);

    for(int idx = 0; idx < numScans; idx++)
    {
      os << std::setw(10) << std::setfill('0') << idx;
      std::string fn = lidarDataPath + "/" + os.str() + ".bin";
      debug_print("%s", fn.c_str());
      os.str(""); os.clear();

      PclPcXYZ pcXyz;
      std::vector<float> intensity;

      if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

      cv::Mat *ri;
      cv::Mat nRi, dnRi;
      double riMax, riMin;

      st = getTsNow();
      ri = riConverter.convertPc2Ri(pcXyz); // TODO: need to encode intensity also
      et = getTsNow();
      pc2ri->info("PC2RI exe\t{}", et-st);

      st = getTsNow();
      riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
      et = getTsNow();
      ri2nri->info("RI2NRI exe\t{}", et-st);


      RunLengthCompressor rleCompressor;
      std::vector<char> compressedRi, compressednRi;
      std::vector<char> decompressedRi, decompressednRi;

      // dict Ri
      st = getTsNow();
      compressedRi = rleCompressor.encode((char*)ri->data, ri->elemSize()*ri->total());
      et = getTsNow();
      ri2dict->info("RI2DICT exe, size\t{}\t{}", et-st, compressedRi.size());

      // dict nRi
      st = getTsNow();
      compressednRi = rleCompressor.encode((char*)nRi.data, nRi.elemSize()*nRi.total());
      et = getTsNow();
      nri2dict->info("NRI2DICT exe, size\t{}\t{}", et-st, compressednRi.size());


      // ridict Ri
      st = getTsNow();
      decompressedRi = rleCompressor.decode(compressedRi, ri->elemSize()*ri->total());
      et = getTsNow();
      ridict2ri->info("RIDICT2RI exe\t{}", et-st);

      // nridict nRi
      st = getTsNow();
      decompressednRi = rleCompressor.decode(compressednRi, nRi.elemSize()*nRi.total());
      et = getTsNow();
      nridict2nri->info("NRIDICT2NRI exe\t{}", et-st);

      cv::Mat recRi(ri->rows, ri->cols, CV_32FC1, decompressedRi.data());
      cv::Mat recNri(nRi.rows, nRi.cols, CV_8UC1, decompressednRi.data());
      cv::Mat recRiFromNri;

      st = getTsNow();
      riConverter.denormalizeRi(&recNri, riMax, &recRiFromNri);
      et = getTsNow();
      nri2ri->info("NRI2RI exe\t{}", et-st);


      PclPcXYZ recPc1, recPc2;

      st = getTsNow();
      recPc1 = riConverter.reconstructPcFromRi(&recRi);
      et = getTsNow();
      ri2pc->info("RI2PC exe\t{}", et-st);

      st = getTsNow();
      recPc2 = riConverter.reconstructPcFromRi(&recRiFromNri);
      et = getTsNow();
      ri2pc->info("RI2PC exe\t{}", et-st);

      float PSNR = calcPSNR(pcXyz, recPc1, 80);
      float CD   = calcCD(pcXyz, recPc1);
      float PSNR2 = calcPSNR(pcXyz, recPc2, 80);
      float CD2   = calcCD(pcXyz, recPc2);

      metricLogger->info("\t\t{}\t{}", PSNR, CD);
      metricLogger2->info("\t\t{}\t{}", PSNR2, CD2);

      visualizer.setViewer(recPc1);
      visualizer.show(1);
      visualizer.setViewer(recPc2);
      visualizer.show(1);

      dnRi.release();
      nRi.release();

      recPc1->clear();
      recPc2->clear();

      ri->release();
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
