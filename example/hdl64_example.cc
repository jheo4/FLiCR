#include <3dpcc>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/PointCloud2.h>

using namespace std;

int main() {
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

  HDL64PCReader hdl64PCReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RIConverter;
  Encoder jpegEncoder;
  jpegEncoder.init("mjpeg", 4500, 64, 40000, 30);
  Decoder jpegDecoder;
  jpegDecoder.init("mjpeg", 4500, 64);

  uint32_t seq = 0;
  while(1) {
    // get pc
    std::vector<HDL64PointCloud> *pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(*pc);

    // convert pc into ri
    //cv::Mat *ri = hdl64RIConverter.convertPC2RIwithXYZ(pc);
    cv::Mat *ri = hdl64RIConverter.convertPC2RI(pc);
    debug_print("RI info: %dx%d", ri->cols, ri->rows);

    std::vector<HDL64PointCloud> *pc2 = hdl64RIConverter.convertRI2PC(ri);
    cv::Mat *ri2 = hdl64RIConverter.convertPC2RI(pc2);
    debug_print("RI2 info: %dx%d", ri2->cols, ri2->rows);

    //cv::imwrite("img/ri_" + to_string(seq) + ".png", *ri);

    //cv::imshow("test", *ri);
    //int k = cv::waitKey(1);

    cv::Mat normRi;
    cv::normalize(*ri2, normRi, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("img/normri_" + to_string(seq) + ".png", normRi);
    //cv::imshow("test", *ri);
    //int k = cv::waitKey(1);

    ///*
    cv::Mat yuvRi = jpegEncoder.gray2yuv(normRi);
    AVPacket pkt;
    av_init_packet(&pkt);
    jpegEncoder.encodeYUV(yuvRi, pkt);
    //jpegEncoder.saveAsFile(pkt, "img/ri_" + to_string(seq) + ".jpg");

    cv::Mat yuvDecFrame;
    jpegDecoder.decodeYUV(pkt.data, pkt.size, yuvDecFrame);
    av_packet_unref(&pkt);
    cv::Mat grayDecFrame = jpegDecoder.yuv2gray(yuvDecFrame);
    cv::imshow("test", grayDecFrame);
    int k = cv::waitKey(1);
    jpegDecoder.saveAsFile(grayDecFrame, "img/ri_" + to_string(seq) + ".png");
    //*/

    seq++;

    sleepMS(50);

    pc->clear();
    delete pc;
    pc2->clear();
    delete pc2;

    ri->release();
    delete ri;
    ri2->release();
    delete ri2;
  }

  return 0;
}