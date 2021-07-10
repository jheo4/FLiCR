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

  std::string kittiBagFile = config["kitti_bag_file"].as<std::string>();
  std::string kittiLeftColImage = config["kitti_left_col_image_topic"].as<std::string>();
  std::string kittiVelodyne = config["kitti_velodyne_topic"].as<std::string>();

  int riRow = (int)(HDL64_VERTICAL_DEGREE / HDL64_THETA_PRECISION);
  int riCol = (int)(HDL64_HORIZONTAL_DEGREE / HDL64_PI_PRECISION);


  /* Set classes */
  HDL64PCReader hdl64PCReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RIConverter;
  Encoder jpegEncoder;
  jpegEncoder.init("mjpeg", riCol, riRow, 40000, 30);
  Decoder jpegDecoder;
  jpegDecoder.init("mjpeg", riCol, riRow);

  double st, et, e2e;

  debug_print("Num of Frames: %d", hdl64PCReader.getNumMsg());

  /* Each Frame Process */
  for(int seq = 0; seq < hdl64PCReader.getNumMsg(); seq++) {
    e2e = 0;

    /* PC Read */
    std::vector<HDL64PointCloud> *pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(*pc);

    //cv::Mat *ri = hdl64RIConverter.convertPC2RIwithXYZ(pc);

    /* PC -> RI */
    st = getTsNow();
    cv::Mat *ri = hdl64RIConverter.convertPC2RI(pc);
    et = getTsNow();
    debug_print("RI info: %dx%d, convTime(%f ms)", ri->cols, ri->rows, et-st);
    e2e += (et - st);
    hdl64RIConverter.getRIConvError(pc, ri);

    // std::vector<HDL64PointCloud> *pc2 = hdl64RIConverter.convertRI2PC(ri);
    // cv::Mat *ri2 = hdl64RIConverter.convertPC2RI(pc2);
    //cv::imwrite("img/ri_" + to_string(seq) + ".png", *ri);
    //cv::imshow("test", *ri);
    //int k = cv::waitKey(1);


    /* RI -> nRI */
    cv::Mat normRi;

    st = getTsNow();
    int max = hdl64RIConverter.normRi(ri, &normRi);
    et = getTsNow();
    debug_print("normTime(%f ms)", et-st);
    e2e += (et - st);
    //cv::imwrite("img/normri_" + to_string(seq) + ".png", normRi);
    //cv::imshow("test", normRi);
    //int k = cv::waitKey(1);

    hdl64RIConverter.getRIQuantError(ri, &normRi, max); // Ri->nRI quan error

    ///*
    AVPacket pkt;
    av_init_packet(&pkt);

    /* nRI -> encoded nRI */
    st = getTsNow();
    cv::Mat yuvRi = jpegEncoder.gray2yuv(normRi);
    jpegEncoder.encodeYUV(yuvRi, pkt);
    et = getTsNow();
    e2e += (et - st);
    debug_print("encTime(%f ms) // E2E (%f ms)", et-st, e2e);

    //jpegEncoder.saveAsFile(pkt, "img/ri_" + to_string(seq) + ".jpg");

    /* encoded nRI -> decoded nRI */
    cv::Mat yuvDecFrame;

    e2e = 0;
    st = getTsNow();
    jpegDecoder.decodeYUV(pkt.data, pkt.size, yuvDecFrame);
    av_packet_unref(&pkt);
    cv::Mat grayDecFrame = jpegDecoder.yuv2gray(yuvDecFrame);
    et = getTsNow();
    e2e += (et - st);
    debug_print("decTime(%f ms)", et-st);


    cv::imshow("test", grayDecFrame);
    int k = cv::waitKey(1);
    //jpegDecoder.saveAsFile(grayDecFrame, "img/ri_" + to_string(seq) + ".png");

    st = getTsNow();
    cv::Mat dnRi;
    hdl64RIConverter.denormRi(&grayDecFrame, max, &dnRi);
    std::vector<HDL64PointCloud> *decPc = hdl64RIConverter.convertRI2PC(&dnRi);
    et = getTsNow();
    e2e += (et - st);
    debug_print("ri2pc (%f ms) // E2E (%f ms), error (%ld)", et-st, e2e, pc->size() - decPc->size());

    jpegDecoder.saveAsFile(grayDecFrame, "img/ri_" + to_string(seq) + ".png");

    decPc->clear();
    delete decPc;

    //*/

    sleepMS(50);

    pc->clear();
    delete pc;
    //pc2->clear();
    //delete pc2;

    ri->release();
    delete ri;
    //ri2->release();
    //delete ri2;
  }

  return 0;
}
