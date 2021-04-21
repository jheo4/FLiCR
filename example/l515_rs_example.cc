#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <librealsense2/rs.hpp>
#include "example.h"

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

  //window app(1280, 720, "RealSense Pointcloud Example");
  auto pipe = std::make_shared<rs2::pipeline>();
  rs2::config rsConfig;
  std::cout << d435iBagFile << std::endl;
  rsConfig.enable_device_from_file(d435iBagFile);
  pipe->start(rsConfig);

  for(int i = 0; i < 300; i++) {
    rs2::frameset frameset = pipe->wait_for_frames();
    //rs2::depth_frame depthFrame = frameset.get_depth_frame().apply_filter(color_map);
    rs2::video_frame colorFrame = frameset.get_color_frame();
    rs2::depth_frame depthFrame = frameset.get_depth_frame();

    const int w = depthFrame.as<rs2::video_frame>().get_width();
    const int h = depthFrame.as<rs2::video_frame>().get_height();

    std::cout << colorFrame.get_width() << "x" << colorFrame.get_height() << std::endl;
    std::cout << w << "x" << h << std::endl << std::endl;
    //cv::Mat depthCVImage(cv::Size(w, h), CV_8UC3, (void*)depthFrame.get_data(), cv::Mat::AUTO_STEP);
    //imshow("test", depthCVImage);
    //int inKey = cv::waitKey(1) & 0xFF;
  }

  return 0;
}
