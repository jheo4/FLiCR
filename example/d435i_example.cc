#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

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
  std::cout << d435iBagFile << std::endl;
  std::cout << d435iColImage << std::endl;
  std::cout << d435iDepth << std::endl;

  // software pipe for reading bag file
  auto pipe = std::make_shared<rs2::pipeline>();;
  rs2::config rsConfig;
  rsConfig.enable_device_from_file(d435iBagFile);
  pipe->start(rsConfig);

  // pointcloud gl app
  window app(WIDTH, HEIGHT, "D435i Example");
  glfw_state appState;
  register_glfw_callbacks(app, appState);
  rs2::pointcloud pointCloud;
  rs2::points points;

  while(app) {
    rs2::frameset frameset = pipe->wait_for_frames();
    rs2::video_frame colorFrame = frameset.get_color_frame();
    rs2::depth_frame depthFrame = frameset.get_depth_frame();

    pointCloud.map_to(colorFrame);
    points = pointCloud.calculate(depthFrame);

    appState.tex.upload(colorFrame);
    draw_pointcloud(app.width(), app.height(), appState, points);
  }

  return 0;
}
