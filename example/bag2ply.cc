#include <3dpcc>

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
  double st, et, e2e;

  debug_print("Num of Frames: %d", hdl64PCReader.getNumMsg());

  /* Each Frame Process */
  for(int seq = 0; seq < hdl64PCReader.getNumMsg(); seq++) {
    e2e = 0;

    /* PC Read */
    PclPcXYZI pc = hdl64PCReader.getNextPCI();
    if(pc == nullptr) break;
    hdl64PCReader.printPCInfo(pc);

    pc->clear();
  }

  return 0;
}

