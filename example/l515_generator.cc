#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt32.h>

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
  std::string l515IMU = config["l515_imu_topic"].as<std::string>();


  rosbag::Bag l515Bag;
  std::vector<std::string> l515BagTopics;
  rosbag::View *view;

  l515Bag.open(l515BagFile, rosbag::bagmode::Read);
  if(!l515Bag.isOpen()) {
    std::cout << "Bag file reading error" << std::endl;
    return 0;
  }

  rosbag::Bag std515;
  std515.open("/home/jin/github/3D_PCC/resources/l515.bag", rosbag::bagmode::Write);
  ros::Time::init();
  ros::Time timer = ros::Time::now();
  ros::Duration duration(1.0f/30);

  // single view with the assumption of synchronized topics
  l515BagTopics.push_back(l515ColImage);
  l515BagTopics.push_back(l515Depth);
  l515BagTopics.push_back(l515IMU);
  view = new rosbag::View(l515Bag, rosbag::TopicQuery(l515BagTopics));

  std_msgs::UInt32 i;
  i.data = 3;
  std515.write("/file_version", ros::Time(timer), i);

  for(rosbag::View::iterator curMsg = view->begin(); curMsg != view->end(); curMsg++) {
    if(curMsg->getTopic() == l515ColImage) {
      sensor_msgs::Image::ConstPtr colorImage = curMsg->instantiate<sensor_msgs::Image>();
      if(colorImage != nullptr) {
        std515.write("/device_0/sensor_1/Color_0/image/data", ros::Time(timer), colorImage);
      }
    }
    else if(curMsg->getTopic() == l515Depth) {
      sensor_msgs::PointCloud2::ConstPtr depthImage = curMsg->instantiate<sensor_msgs::PointCloud2>();
      if(depthImage != nullptr) {
        std515.write("/device_0/sensor_0/Depth_0/pointcloud/data", ros::Time(timer), depthImage);
      }
    }
    else if(curMsg->getTopic() == l515IMU) {
      sensor_msgs::ImuConstPtr imuData = curMsg->instantiate<sensor_msgs::Imu>();
      if(imuData != nullptr) {
        std515.write("/device_0/sensor_2/Gyro_0/imu/data", ros::Time(timer), imuData);
      }
    }
  }

  std515.close();
  return 0;
}
