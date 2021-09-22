#include <3dpcc>

using namespace std;

PclXYZ convRiPixToPoint(int y, int x, float rho)
{
  float thetaPrecision = HDL64_THETA_PRECISION;
  float piPrecision = HDL64_PI_PRECISION;

  float nTheta = (y * thetaPrecision);
  float nPi = (x * piPrecision);

  float theta = nTheta - HDL64_VERTICAL_DEGREE_OFFSET;
  float pi    = nPi    - HDL64_HORIZONTAL_DEGREE_OFFSET;

  float rTheta = theta * PI/180.0f;
  float rPi    = pi    * PI/180.0f;

  PclXYZ p;
  p.x = rho * std::sin(rTheta) * std::cos(rPi);
  p.y = rho * std::sin(rTheta) * std::sin(rPi);
  p.z = rho * std::cos(rTheta);

  return p;
}

int main() {
  double st, et;

  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config         = YAML::LoadFile(configYaml);
  std::string kittiBagFile  = config["kitti_bag_file"].as<std::string>();
  std::string kittiVelodyne = config["kitti_velodyne_topic"].as<std::string>();
  std::string kittiImu      = config["kitti_imu_topic"].as<std::string>();
  std::string kittiGps      = config["kitti_gps_topic"].as<std::string>();

  /* Set classes */
  HDL64PCReader hdl64PcReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RiConverter;

  OxtsReader oxtsReader(kittiBagFile, kittiImu, kittiGps);

  PclPcXYZ pc1, pc2, predictedPc2;
  OxtsMsg oxtsMsg;

  pc1     = hdl64PcReader.getNextPC();
  pc2     = hdl64PcReader.getNextPC();
  oxtsMsg = oxtsReader.getNextMsg();
  oxtsMsg.print();

  cv::Mat *ri1  = hdl64RiConverter.convertPc2Ri(pc1);
  cv::Mat *ri2  = hdl64RiConverter.convertPc2Ri(pc2);

  cv::Mat nRi1, nRi2;
  double riMax1, riMax2;

  hdl64RiConverter.normalizeRi(ri1, &nRi1, &riMax1);
  hdl64RiConverter.normalizeRi(ri2, &nRi2, &riMax2);

  /*
  cv::Mat cropNRi1, cropNRi2;
  cropNRi1 = nRi1(cv::Range(0, 20), cv::Range(200, 250));
  cropNRi2 = nRi2(cv::Range(0, 20), cv::Range(200, 250));
  cropNRi1.at<uint8_t>(10, 19) = 255; cropNRi2.at<uint8_t>(10, 6) = 255;
  */
  // 10, 214-217: 39
  // 10, 203-205: 41
  PclXYZ p1 = convRiPixToPoint(10, 217, ri1->at<float>(10, 214));
  PclXYZ p2 = convRiPixToPoint(10, 205, ri2->at<float>(10, 203));

  VecXyz vel;
  vel.x = (p2.x-p1.x) * KITTI_DATASET_FREQUENCY; vel.y = (p2.y-p1.y) * KITTI_DATASET_FREQUENCY; vel.z = (p2.z-p1.z) * KITTI_DATASET_FREQUENCY;

  PcPredictor predictor;
  predictedPc2 = predictor.predictNextPc(pc1, vel);

  cv::Mat *pRi2 = hdl64RiConverter.convertPc2Ri(predictedPc2);
  cv::Mat nPredRi2;
  double predRiMax2;
  hdl64RiConverter.normalizeRi(pRi2, &nPredRi2, &predRiMax2);


  cv::Mat nDiff1 = nRi2 - nRi1;
  cv::Mat nDiff2 = nRi1 - nRi2;
  cv::Mat pDiff1 = nRi2 - nPredRi2;
  cv::Mat pDiff2 = nPredRi2 - nRi2;

  debug_print("%d vs %d", cv::countNonZero(nDiff1)+cv::countNonZero(nDiff2), cv::countNonZero(pDiff1)+cv::countNonZero(pDiff2));


  while(1)
  {
    cv::imshow("nRi1", nRi1);
    cv::imshow("nRi2", nRi2);
    cv::imshow("nPredRi2", nPredRi2);
    cv::waitKey(10);
  }

  return 0;
}

