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
  HDL64RIConverter hdl64RiConverter;
  Visualizer pcVisualizer;
  pcVisualizer.initViewerXYZ();
  double st, et, e2e;

  debug_print("Num of Frames: %d", hdl64PCReader.getNumMsg());

  /* Each Frame Process */
  for(int seq = 0; seq < hdl64PCReader.getNumMsg(); seq++) {
    e2e = 0;

    /* PC Read */
    PclPcXYZ pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(pc);


    /* PC-RI-nRI-RI-riPC */
    cv::Mat *ri = hdl64RiConverter.convertPc2Ri(pc);
    cv::Mat nRi;
    double riMax;
    hdl64RiConverter.normalizeRi(ri, &nRi, &riMax);
    hdl64RiConverter.calcRiPixNormError(ri, riMax, &nRi);

    cv::Mat riReconstructed;
    hdl64RiConverter.denormalizeRi(&nRi, riMax, &riReconstructed);

    PclPcXYZ pcReconstructed = hdl64RiConverter.reconstructPcFromRi(&riReconstructed);

    /* riPC Visualization */
    pcVisualizer.setViewer(pcReconstructed);
    for(int i = 0; i < 1; i++) {
      pcVisualizer.show(100);
      pcVisualizer.saveToFile("pc_" + std::to_string(riCol) + "_" + std::to_string(seq) + ".png");
    }

    pc->clear();
    ri->release();
    pcReconstructed->clear();
  }

  return 0;
}

