#include <3dpcc>

using namespace std;

int main() {
  double st, et;
  int riRow = 64;
  int riCol = 4500;

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

  PcReader pcReader;
  HDL64RIConverter riConverter;
  Encoder encoder;
  Decoder decoder;
  encoder.init(encName, riCol, riRow, br, fps, qp, crf);
  decoder.init(decName, riCol, riRow);

  Visualizer visualizer;
  visualizer.initViewerXYZ();

  int numScans = 1;

  for(int idx = 0; idx < numScans; idx++)
  {
    std::string fn = "/home/mnt/github/3D_PCC/build/0000000000.bin";

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


    AVPacket pkt;
    av_init_packet(&pkt);

    st = getTsNow();
    cv::Mat yuvRi = encoder.gray2yuv(nRi);
    encoder.encodeYUV(yuvRi, pkt);
    et = getTsNow();


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


      st = getTsNow();
      riConverter.denormalizeRi(&decNri, riMax, &decRi);
      decXyz = riConverter.reconstructPcFromRi(&decRi);
      et = getTsNow();


      // metric logging
      float SE   = calcSamplingError(pcXyz, decXyz);
      float PSNR = calcPSNR(pcXyz, decXyz, 80);
      float CD   = calcCD(pcXyz, decXyz);
      debug_print("SE(%f), PSNR(%f), CD(%f)", SE, PSNR, CD);

      visualizer.setViewer(decXyz);
      visualizer.show(10000);


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
