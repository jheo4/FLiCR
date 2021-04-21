#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <librealsense2/rs.hpp>
#include <opencv/cv.hpp>

int main() {
  // Getting the configs from yaml
  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config = YAML::LoadFile(configYaml);

  std::string l515BagFile = config["l515_bag_file"].as<std::string>();
  std::string l515ColImage = config["l515_col_image_topic"].as<std::string>();
  std::string l515Depth = config["l515_dep_point_topic"].as<std::string>();


  rosbag::Bag l515Bag;
  std::vector<std::string> l515BagTopics;
  rosbag::View *view;

  l515Bag.open(l515BagFile, rosbag::bagmode::Read);
  if(!l515Bag.isOpen()) {
    std::cout << "Bag file reading error" << std::endl;
    return 0;
  }

  // single view with the assumption of synchronized topics
  l515BagTopics.push_back(l515ColImage);
  l515BagTopics.push_back(l515Depth);
  view = new rosbag::View(l515Bag, rosbag::TopicQuery(l515BagTopics));

  for(rosbag::View::iterator curMsg = view->begin(); curMsg != view->end(); curMsg++) {
    if(curMsg->getTopic() == l515ColImage) {
      sensor_msgs::Image::ConstPtr colorImage = curMsg->instantiate<sensor_msgs::Image>();
      if(colorImage != nullptr) {
        std::cout << colorImage->height << "x" << colorImage->width << std::endl;
        cv::Mat colorCVImg(cv::Size(1280, 720), CV_8UC3, (void*)colorImage->data.data(), cv::Mat::AUTO_STEP);
        imshow("Color Image", colorCVImg);
        int inKey = cv::waitKey(1) & 0xFF;
      }
    }
    else if(curMsg->getTopic() == l515Depth) {
      sensor_msgs::PointCloud2::ConstPtr depthImage = curMsg->instantiate<sensor_msgs::PointCloud2>();
      if(depthImage != nullptr) {
        std::cout << depthImage->height << "x" << depthImage->width << std::endl;
        // | x | y | z | intensity |
      }
    }
  }

  return 0;
}
