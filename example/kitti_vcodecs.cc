#include <3dpcc>

using namespace std;

int main() {
  double st, et;
  int riRow = (int)(HDL64_VERTICAL_DEGREE   / HDL64_THETA_PRECISION);
  int riCol = (int)(HDL64_HORIZONTAL_DEGREE / HDL64_PI_PRECISION);

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
  std::string encName       = config["enc"].as<std::string>();
  std::string decName       = config["dec"].as<std::string>();
  int br                    = config["br"].as<int>();
  int fps                   = config["fps"].as<int>();
  int qp                    = config["qp"].as<int>();
  int crf                   = config["crf"].as<int>();


  std::shared_ptr<spdlog::logger> pc2nri       = spdlog::basic_logger_st("pc2nri", "logs/"+dataCategory+"/enc_pc2nri.log");
  std::shared_ptr<spdlog::logger> nri2pc       = spdlog::basic_logger_st("nri2pc", "logs/"+dataCategory+"/dec_nri2pc.log");
  std::shared_ptr<spdlog::logger> encLogger    = spdlog::basic_logger_st("encLogger", "logs/"+dataCategory+"/enc.log");
  std::shared_ptr<spdlog::logger> decLogger    = spdlog::basic_logger_st("decLogger", "logs/"+dataCategory+"/dec.log");
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/"+dataCategory+"/metric.log");

  encLogger->info("Encoder Info: {}, br {}, fps {}, qp {}, crf {}", encName, br, fps, qp, crf);
  decLogger->info("Decoder Info: {}", decName);
  metricLogger->info("\tSampling Error\tpcPSNR\triPSNR\triSSIM\tnriSSIM");


  std::ostringstream os;
  PcReader pcReader;
  HDL64RIConverter riConverter;
  Encoder encoder;
  Decoder decoder;
  encoder.init(encName, riCol, riRow, br, fps, qp, crf);
  decoder.init(decName, riCol, riRow);

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
    while(ent = readdir(dir)) numScans++;
  }
  closedir(dir);
  debug_print("# of scans: %d", numScans);

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    PclPcXYZ pcXyz;
    std::vector<float> intensity;
    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) break;

    /* pc -> ri -> nRi -> yuv -> encoded bytes */
    cv::Mat *ri;
    cv::Mat nRi;
    double riMax, riMin;

    st = getTsNow();
    ri = riConverter.convertPc2Ri(pcXyz); // TODO: need to encode intensity also
    riConverter.normalizeRi(ri, &nRi, &riMin, &riMax);
    et = getTsNow();
    pc2nri->info("PC2nRI exe\t{}", et-st);


    AVPacket pkt;
    av_init_packet(&pkt);

    st = getTsNow();
    cv::Mat yuvRi = encoder.gray2yuv(nRi);
    encoder.encodeYUV(yuvRi, pkt);
    et = getTsNow();
    encLogger->info("encoding exe / byte size\t{}\t{}", et-st, pkt.size);


    /* encoded bytes -> dec_yuv -> dec_nRi -> dec_ri -> dec_pc */
    if(pkt.size > 0) {
      AVPacket decodingPkt;
      av_packet_from_data(&decodingPkt, pkt.data, pkt.size);
      decodingPkt.side_data_elems = 0;

      cv::Mat decYuv, decNri, decRi;
      PclPcXYZ decXyz;

      st = getTsNow();
      decoder.decodeYUV(decodingPkt, decYuv);
      decNri = decoder.yuv2gray(decYuv);
      et = getTsNow();
      decLogger->info("decoding exe\t{}", et-st);


      st = getTsNow();
      riConverter.denormalizeRi(&decNri, riMax, &decRi);
      decXyz = riConverter.reconstructPcFromRi(&decRi);
      et = getTsNow();
      nri2pc->info("nRI2PC exe\t{}", et-st);


      // metric logging
      float samplingError = riConverter.calcRiQuantError(pcXyz, ri);
      float PSNR = calcPSNR(pcXyz, decXyz, 80);
      float riPSNR = getImgPSNR(*ri, decRi, riMax);
      float riMSSIM1 = getImgMSSIM(nRi, decNri);
      float riMSSIM2 = getImgMSSIM(*ri, decRi);

      metricLogger->info("\t{}\t{}\t{}\t{}\t{}", samplingError, PSNR, riPSNR, riMSSIM1, riMSSIM2);

      visualizer.setViewer(decXyz);
      visualizer.show(1);


      decRi.release();
      decNri.release();
      decXyz->clear();
    }

    ri->release(); delete ri;
    pcXyz->clear();
    intensity.clear();


    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}
