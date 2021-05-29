#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt32.h>

#include <librealsense2/rs.hpp>
#include "example.h"

#define WIDTH 1280
#define HEIGHT 720

int main() {
  // Getting the configs from yaml
  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node yamlConfig = YAML::LoadFile(configYaml);

  std::string d435iBagFile = yamlConfig["d435i_bag_file"].as<std::string>();
  std::string d435iColImage = yamlConfig["d435i_col_image_topic"].as<std::string>();
  std::string d435iDepth = yamlConfig["d435i_depth_image_topic"].as<std::string>();
  std::string d435iVersion = "/file_version";
  std::cout << d435iBagFile << std::endl;
  std::cout << d435iColImage << std::endl;
  std::cout << d435iDepth << std::endl;

  rosbag::Bag d435iBag;
  std::vector<std::string> d435iBagTopics;
  rosbag::View *view;

  d435iBag.open(d435iBagFile, rosbag::bagmode::Read);
  if(!d435iBag.isOpen()) {
    std::cout << "Bag file reading error" << std::endl;
    return 0;
  }

  // single view with the assumption of synchronized topics
  d435iBagTopics.push_back(d435iColImage);
  d435iBagTopics.push_back(d435iDepth);
  d435iBagTopics.push_back(d435iVersion);
  view = new rosbag::View(d435iBag, rosbag::TopicQuery(d435iBagTopics));

  for(rosbag::View::iterator curMsg = view->begin(); curMsg != view->end(); curMsg++) {
    if(curMsg->getTopic() == d435iColImage) {
      sensor_msgs::Image::ConstPtr colorImage = curMsg->instantiate<sensor_msgs::Image>();
      if(colorImage != nullptr) {
        std::cout << colorImage->height << "x" << colorImage->width << std::endl;
        //cv::Mat colorCVImg(cv::Size(1280, 720), CV_8UC3, (void*)colorImage->data.data(), cv::Mat::AUTO_STEP);
        //imshow("Color Image", colorCVImg);
        //int inKey = cv::waitKey(1) & 0xFF;
      }
    }
    else if(curMsg->getTopic() == d435iDepth) {
      sensor_msgs::PointCloud2::ConstPtr depthImage = curMsg->instantiate<sensor_msgs::PointCloud2>();
      if(depthImage != nullptr) {
        std::cout << depthImage->height << "x" << depthImage->width << std::endl;
        std::cout << depthImage->point_step << ", " << depthImage->row_step << std::endl;
        // | x | y | z | intensity |
      }
    }
    else if(curMsg->getTopic() == d435iVersion) {
      //std_msgs/UInt32
      std_msgs::UInt32ConstPtr versionNum = curMsg->instantiate<std_msgs::UInt32>();
      if(versionNum != nullptr) {
        std::cout << "Version Info: " << versionNum->data << std::endl;
      }
    }

  }

  return 0;
}
