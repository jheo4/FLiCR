#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <types.h>

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
  std::string kittiLeftColInfo = config["kitti_left_col_info_topic"].as<std::string>();
  std::string kittiLeftColImage = config["kitti_left_col_image_topic"].as<std::string>();
  std::string kittiVelodyne = config["kitti_velodyne_topic"].as<std::string>();

  rosbag::Bag kittiBag;
  std::vector<std::string> kittiBagTopics;
  rosbag::View *view;

  kittiBag.open(kittiBagFile, rosbag::bagmode::Read);
  if(!kittiBag.isOpen()) {
    std::cout << "Bag file reading error" << std::endl;
    return 0;
  }

  // single view with the assumption of synchronized topics
  kittiBagTopics.push_back(kittiLeftColInfo);
  kittiBagTopics.push_back(kittiLeftColImage);
  kittiBagTopics.push_back(kittiVelodyne);
  view = new rosbag::View(kittiBag, rosbag::TopicQuery(kittiBagTopics));

  for(rosbag::View::iterator curMsg = view->begin(); curMsg != view->end(); curMsg++) {
    if(curMsg->getTopic() == kittiLeftColImage) {
      sensor_msgs::Image::ConstPtr leftColImage = curMsg->instantiate<sensor_msgs::Image>();
      if(leftColImage != nullptr) {
        std::cout << "leftColImage" << std::endl;
        cout << "\t" << leftColImage->width << "x" << leftColImage->height << endl;
      }
    }
    else if(curMsg->getTopic() == kittiVelodyne) {
      sensor_msgs::PointCloud2::ConstPtr velodynePointCloud = curMsg->instantiate<sensor_msgs::PointCloud2>();
      if(velodynePointCloud != nullptr) {
        HDL64PointCloud *pc = (HDL64PointCloud*)velodynePointCloud->data.data();
        std::cout << "velodynePointCloud" << std::endl;
        cout << "\t" << velodynePointCloud->width << "x" << velodynePointCloud->height << endl;;
        for(int i = 0; i < 10; i++) {
          pc[i*i].print();
        }
      }
    }
  }

  return 0;
}
