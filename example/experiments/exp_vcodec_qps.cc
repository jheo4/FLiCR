#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
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

  std::string encName       = config["enc"].as<std::string>();
  std::string decName       = config["dec"].as<std::string>();
  int br                    = config["br"].as<int>();
  int fps                   = config["fps"].as<int>();
  int qps[5] = {0, 5, 10, 15, 20};

  float riPrecisions[5] = {HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048, HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  numScans = 100;

  for(int i = 0; i < 5; i++)
  {
    riRow = (int)(HDL64_VERTICAL_DEGREE   / HDL64_THETA_PRECISION);
    riCol = (int)(HDL64_HORIZONTAL_DEGREE / riPrecisions[i]);

    for(int j = 0; j < 5; j++)
    {
      std::shared_ptr<spdlog::logger> pc2nri       = spdlog::basic_logger_st("pc2nri_"+to_string(riCol)+"_"+to_string(qps[j]),
          "logs/"+dataCategory+"/"+to_string(riCol)+"_"+to_string(qps[j])+"/enc_pc2nri.log");
      std::shared_ptr<spdlog::logger> nri2pc       = spdlog::basic_logger_st("nri2pc"+to_string(riCol)+"_"+to_string(qps[j]),
          "logs/"+dataCategory+"/"+to_string(riCol)+"_"+to_string(qps[j])+"/dec_nri2pc.log");
      std::shared_ptr<spdlog::logger> encLogger    = spdlog::basic_logger_st("encLogger"+to_string(riCol)+"_"+to_string(qps[j]),
          "logs/"+dataCategory+"/"+to_string(riCol)+"_"+to_string(qps[j])+"/enc.log");
      std::shared_ptr<spdlog::logger> decLogger    = spdlog::basic_logger_st("decLogger"+to_string(riCol)+"_"+to_string(qps[j]),
          "logs/"+dataCategory+"/"+to_string(riCol)+"_"+to_string(qps[j])+"/dec.log");
      std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger"+to_string(riCol)+"_"+to_string(qps[j]),
          "logs/"+dataCategory+"/"+to_string(riCol)+"_"+to_string(qps[j])+"/metric.log");

      encLogger->info("Encoder Info: {}, br {}, fps {}, qp {}", encName, br, fps, qps[j]);
      decLogger->info("Decoder Info: {}", decName);
      metricLogger->info("\tSamplingError\tPSNR\tCD");

      std::ostringstream os;
      PcReader pcReader;
      RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                        HDL64_THETA_PRECISION, riPrecisions[i],
                        HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                        HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

      Encoder encoder;
      Decoder decoder;
      encoder.init(encName, riCol, riRow, br, fps, -1, qps[j]);
      decoder.init(decName, riCol, riRow);

      for(int idx = 0; idx < numScans; idx++)
      {
        os << std::setw(10) << std::setfill('0') << idx;
        std::string fn = lidarDataPath + "/" + os.str() + ".bin";
        os.str(""); os.clear();

        types::PclPcXyz pcXyz = pcReader.readXyzFromXyziBin(fn);

        /* pc -> ri -> nRi -> yuv -> encoded bytes */
        cv::Mat ri;
        cv::Mat nRi;
        double riMax, riMin;

        st = getTsNow();
        riConverter.convertPc2Ri(pcXyz, ri, true);
        riConverter.normalizeRi(ri, nRi, riMin, riMax);
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
          types::PclPcXyz decXyz;

          st = getTsNow();
          decoder.decodeYUV(decodingPkt, decYuv);
          decNri = decoder.yuv2gray(decYuv);
          et = getTsNow();
          decLogger->info("decoding exe\t{}", et-st);

          st = getTsNow();
          riConverter.denormalizeRi(decNri, riMin, riMax, decRi);
          decXyz = riConverter.reconstructPcFromRi(decRi, true);
          et = getTsNow();
          nri2pc->info("nRI2PC exe\t{}", et-st);

          // TODO: save DecPC

          // metric logging
          float samplingError = Metrics::calcPoinNumDiffBtwPcs(pcXyz, decXyz);
          float PSNR = Metrics::calcPsnrBtwPcs(pcXyz, decXyz, 80);
          float CD = Metrics::calcCdBtwPcs(pcXyz, decXyz);

          metricLogger->info("\t{}\t{}\t{}", samplingError, PSNR, CD);

          decRi.release();
          decNri.release();
          decXyz->clear();
        }

        ri.release();
        pcXyz->clear();

        printProgress((float)idx/(float)numScans);
      }
      printProgress(1);
    }
  }

  return 0;
}

