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

  uint32_t seq = 0;
  while(1) {
    // get pc
    std::vector<HDL64PointCloud> *pc = hdl64PCReader.getNextPC();
    if(pc == nullptr) break;
    debug_print("PC size: %ld", pc->size());

    // convert pc into ri
    //cv::Mat *ri = hdl64RIConverter.convertPC2RIwithXYZ(pc);
    cv::Mat *ri = hdl64RIConverter.convertPC2RI(pc);
    debug_print("RI info: %dx%d", ri->cols, ri->rows);

    cv::imwrite("img/ri_" + to_string(seq) + ".png", *ri);

    cv::Mat normRi;
    cv::normalize(*ri, normRi, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("img/normri_" + to_string(seq) + ".png", normRi);

    seq++;

    sleepMS(50);

    pc->clear();
    delete pc;

    ri->release();
    delete ri;
  }

  return 0;
}
