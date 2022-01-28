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

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("size_logger", "logs/vid_size/"+dataCategory+"/size.log");

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

    Encoder encoder;
    // libx264 h264_nvenc libx265
    encoder.init("libx264", riConverter.riCol, riConverter.riRow, 4000000, 30, 0, -1);

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

      st = getTsNow();
      riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
      et = getTsNow();


      AVPacket pkt;
      av_init_packet(&pkt);

      cv::Mat yuvRi = encoder.gray2yuv(nRi);
      encoder.encodeYUV(yuvRi, pkt);

      /* encoded bytes -> dec_yuv -> dec_nRi -> dec_ri -> dec_pc */
      if(pkt.size > 0) {
        AVPacket decodingPkt;
        av_packet_from_data(&decodingPkt, pkt.data, pkt.size);
        decodingPkt.side_data_elems = 0;

        metricLogger->info("\t{}", pkt.size, decodingPkt.size);

      }

      dnRi.release();
      nRi.release();

      ri->release();
      nRi.release();

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
