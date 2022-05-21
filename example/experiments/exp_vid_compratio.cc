#include "opencv2/core/hal/interface.h"
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

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("size_logger", "logs/vid_size/"+dataCategory+"/size.log");

  std::ostringstream os;
  PcReader pcReader;

  Visualizer visualizer;
  visualizer.initViewerXyz();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  numScans = 50;
  debug_print("# of scans: %d", numScans);

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048,
                      HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};


  for(int prec = 0; prec < 6; prec++)
  {
    RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, piPrec[prec],
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

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

      types::PclPcXyz pcXyz;
      std::vector<float> intensity;

      if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

      cv::Mat ri;
      cv::Mat nRi, dnRi;
      double riMax, riMin;

      st = getTsNow();
      riConverter.convertPc2Ri(pcXyz, ri, true); // TODO: need to encode intensity also
      et = getTsNow();

      st = getTsNow();
      riConverter.normalizeRi(ri, nRi, riMin, riMax);
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

      ri.release();
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
