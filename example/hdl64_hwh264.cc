#include <3dpcc>

using namespace std;

int main() {
  /* Set configs from yaml */
  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config = YAML::LoadFile(configYaml);

  std::string kittiBagFile      = config["kitti_bag_file"].as<std::string>();
  std::string kittiLeftColImage = config["kitti_left_col_image_topic"].as<std::string>();
  std::string kittiVelodyne     = config["kitti_velodyne_topic"].as<std::string>();

  int riRow = (int)(HDL64_VERTICAL_DEGREE / HDL64_THETA_PRECISION);
  int riCol = (int)(HDL64_HORIZONTAL_DEGREE / HDL64_PI_PRECISION);

  /* Set classes */
  HDL64PCReader hdl64PCReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RIConverter;

  Visualizer visualizer;
  visualizer.initViewerXYZ();

  Encoder encoder;
  encoder.init("h264_nvenc", riCol, riRow, 40000, 30);
  Decoder decoder;
  decoder.init("h264_cuvid", riCol, riRow);

  double st, et, e2e;

  debug_print("Num of Frames: %d", hdl64PCReader.getNumMsg());

  /* Each Frame Process */
  for(int seq = 0; seq < hdl64PCReader.getNumMsg(); seq++) {
    debug_print("\n\n %dth msg", seq);
    e2e = 0;

    /* PC Read */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(pc);

    /* PC -> RI */
    st = getTsNow();
    cv::Mat *ri = hdl64RIConverter.convertPC2RI(pc);
    et = getTsNow();
    debug_print("RI info: %dx%d, convTime(%f ms)", ri->cols, ri->rows, et-st);

    hdl64RIConverter.getRIConvError(pc, ri); // check PC->RI error

    /* RI -> nRI */
    cv::Mat nRi;
    double riMax = hdl64RIConverter.normRi(ri, &nRi);
    hdl64RIConverter.getRIQuantError(ri, riMax, &nRi);

    AVPacket pkt;
    av_init_packet(&pkt);

    /* nRI -> encoded nRI */
    cv::Mat yuvRi = encoder.rgb2yuv(nRi);
    encoder.encodeYUV(yuvRi, pkt);

    debug_print("PKT INFO: size(%d), side_data_elems(%d)", pkt.size, pkt.side_data_elems);

    /* encoded nRI -> decoded nRI */
    if(pkt.size > 0) {
      AVPacket decodingPkt;
      av_packet_from_data(&decodingPkt, pkt.data, pkt.size);
      decodingPkt.side_data_elems = 0;

      cv::Mat yuvDecFrame;

      decoder.decodeYUV(decodingPkt, yuvDecFrame);
      cv::Mat nRiReconstructed = decoder.yuv2rgb(yuvDecFrame);

      cv::imshow("test", nRi);
      cv::imshow("test2", nRiReconstructed);
      int k = cv::waitKey(1);

      cv::Mat riReconstructed;
      hdl64RIConverter.denormRi(&nRiReconstructed, riMax, &riReconstructed);

      PCLPcPtr pcReconstructed = hdl64RIConverter.convertRI2PC(&riReconstructed);

      /* PC Visualization */
      visualizer.setViewer(pcReconstructed);
      for(int i = 0; i < 1; i++) {
        visualizer.show(50);
        visualizer.saveToFile("pc_h264HW_" + std::to_string(seq) + ".png");
      }

      riReconstructed.release();
      nRiReconstructed.release();
      pcReconstructed->clear();
    }

    av_packet_unref(&pkt);
    pc->clear();
    ri->release();
    delete ri;
    nRi.release();
  }

  return 0;
}
