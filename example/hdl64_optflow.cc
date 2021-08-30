#include "opencv2/core.hpp"
#include <3dpcc>

using namespace std;

int main() {
  double st, et;

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

  /* Set classes */
  HDL64PCReader hdl64PCReader(kittiBagFile, kittiVelodyne);
  HDL64RIConverter hdl64RiConverter;
  //Visualizer pcVisualizer;
  //pcVisualizer.initViewerXYZ();

  PclPcXYZ pc1, pc2;

  pc1 = hdl64PCReader.getNextPC();
  pc2 = hdl64PCReader.getNextPC();

  cv::Mat *ri1 = hdl64RiConverter.convertPc2Ri(pc1);
  cv::Mat *ri2 = hdl64RiConverter.convertPc2Ri(pc2);
  cv::Mat d12 = *ri2 - *ri1;

  cv::Mat newRi2 = *ri1 + d12;

  cv::Mat nNewRi2, nRi1, nRi2;
  double newRi2Max, riMax1, riMax2;
  hdl64RiConverter.normalizeRi(&newRi2, &nNewRi2, &newRi2Max);
  hdl64RiConverter.normalizeRi(ri1, &nRi1, &riMax1);
  hdl64RiConverter.normalizeRi(ri2, &nRi2, &riMax2);


  // ND
  cv::Mat absDiff, diff;
  cv::absdiff(nNewRi2, nRi1, absDiff);
  diff = nRi2 - nRi1;

  cv::Scalar tempVal = cv::mean(diff);
  float myMatMean = tempVal.val[0];
  printf("Diff Mean: %f\n", myMatMean);

  cv::Scalar tempVal2 = cv::mean(d12);
  float d12Mean = tempVal2.val[0];
  printf("d12 Mean: %f\n", d12Mean);

  int binCount = 0, diffBinCount = 0;
  for(int row = 0; row < diff.rows; row++)
  {
    for(int col = 0; col < diff.cols; col++)
    {
      if(diff.at<float>(row, col) == 0) binCount++;
      if(d12.at<float>(row, col) == 0) diffBinCount++;
    }
  }
  debug_print("binCount %d", binCount);
  debug_print("diffBinCount %d", diffBinCount);

  double min, max;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(d12, &min, &max, &min_loc, &max_loc);
  debug_print("min/max %f/%f", min, max);


  // SOF
  vector<cv::Scalar> colors;
  cv::RNG rng;
  for(int i = 0; i < 100; i++)
  {
    int r = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int b = rng.uniform(0, 256);
    colors.push_back(cv::Scalar(r,g,b));
  }

  st = getTsNow();
  vector<cv::Point2f> p0, p1;
  cv::goodFeaturesToTrack(nRi1, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
  et = getTsNow();
  debug_print("SOF feat: %f", et-st);

  cv::Mat sofVis = cv::Mat::zeros(nRi1.size(), nRi1.type());

  st = getTsNow();
  std::vector<uchar> status;
  std::vector<float> err;
  cv::TermCriteria terminationCriteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
                                                          10, 0.03);
  cv::calcOpticalFlowPyrLK(nRi1, nRi2, p0, p1, status, err, cv::Size(16, 16), 2, terminationCriteria);
  et = getTsNow();
  debug_print("SOF exe: %f", et-st);

  std::vector<cv::Point2f> newFeatP;
  for(int i = 0; i < p0.size(); i++)
  {
    if(status[i] == 1)
    {
      newFeatP.push_back(p1[i]);
      cv::line(sofVis, p1[i], p0[i], colors[i], 2);
      cv::circle(nRi2, p1[i], 1, colors[i], -1);
    }
  }
  cv::Mat sofFrame;
  cv::add(nRi2, sofVis, sofFrame);

  // DOF
  cv::Mat flow(nRi1.size(), CV_32FC2);
  st = getTsNow();
  cv::calcOpticalFlowFarneback(nRi1, nRi2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  et = getTsNow();
  debug_print("DOF exe: %f", et-st);

  cv::Mat flowParts[2];
  cv::split(flow, flowParts);
  cv::Mat magnitude, angle, nMagnitude;
  cv::cartToPolar(flowParts[0], flowParts[1], magnitude, angle, true);
  cv::normalize(magnitude, nMagnitude, 0.0f, 1.0f, cv::NORM_MINMAX);
  angle *= ((1.f/360.f) * (180.f/255.f));

  cv::Mat _hsv[3], hsv, hsv8, bgr;
  _hsv[0] = angle;
  _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
  _hsv[2] = nMagnitude;
  cv::merge(_hsv, 3, hsv);
  hsv.convertTo(hsv8, CV_8U, 255.0);
  cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

  /*
  cv::Mat hist;
  int hitSize = 500;
  float range[] = { -100, 300 };
  const float* histRange[] = {range};
  cv::calcHist(&diff, 1, 0, cv::Mat(), hist, 1, &hitSize, histRange, true, false);
  */


  while(1)
  {
    cv::imshow("nRi1", nRi1);
    cv::imshow("nRi2", nRi2);
    cv::imshow("absDiff", absDiff);
    cv::imshow("diff", diff);
    cv::imshow("SOF", sofFrame);
    cv::imshow("DOF", bgr);
    cv::waitKey(10);
  }

  /*
  cv::Mat riReconstructed;
  hdl64RiConverter.denormalizeRi(&diff, riMax1, &riReconstructed);

  PclPcXYZ pcReconstructed = hdl64RiConverter.reconstructPcFromRi(&riReconstructed);

  pcVisualizer.setViewer(pcReconstructed);
  for(int i = 0; i < 1; i++) {
    pcVisualizer.show(5000);
    pcVisualizer.saveToFile("diff" + std::to_string(riCol) + "_" + std::to_string(0) + ".png");
  }*/

  pc1->clear();
  pc2->clear();
  ri1->release();
  ri2->release();
  //pcReconstructed->clear();

  return 0;
}

