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

  Visualizer visualizer;
  visualizer.initViewerXYZ();

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(pc);

    /* PC -> RI */
    st = getTsNow();
    cv::Mat *ri = hdl64RIConverter.convertPC2RI(pc);
    et = getTsNow();
    debug_print("RI info: %dx%d, convTime(%f ms)", ri->cols, ri->rows, et-st);

    cv::Mat nRi;
    hdl64RIConverter.normRi(ri, &nRi);
    cv::imshow("test", nRi);
    int k = cv::waitKey(1);

    //hdl64RIConverter.getRIConvError(pc, ri);

    PCLPcPtr pc2 = hdl64RIConverter.convertRI2PC(ri);

    /* PC Visualization */
    visualizer.setViewer(pc);
    for(int i = 0; i < 10; i++) {
      visualizer.show(100);
    }

    pc->clear();
    ri->release();
    delete ri;
  }

  return 0;
}
