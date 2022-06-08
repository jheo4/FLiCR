#include <flicr>

using namespace std;
using namespace flicr;

int main()
{
  RiInterpolator riInterpolator;
  riInterpolator.setIntr(64, 4500, 2, 2);
  riInterpolator.printSetting();

  riInterpolator.setIntr(1, 5, 5, 1);
  cv::Mat testMat(1, 5, CV_8UC1, cv::Scalar(0));
  testMat.at<uchar>(0, 0) = 10;
  testMat.at<uchar>(0, 1) = 0;
  testMat.at<uchar>(0, 2) = 22;
  testMat.at<uchar>(0, 3) = 25;
  testMat.at<uchar>(0, 4) = 0;
  cout << testMat << endl;

  IntrIndexInfo t = riInterpolator.getHorizontalIntrIndex(testMat, 0, 0, RiInterpolator::IntrIndexPolicy::GreatestGradient, 10);
  t.print();


  riInterpolator.setIntr(1, 7, 7, 1);

  cv::Mat testMat1(1, 7, CV_8UC1, cv::Scalar(0));
  cv::Mat testMat11(1, 8, CV_8UC1, cv::Scalar(0));
  testMat1.at<uchar>(0, 0) = 10;
  testMat1.at<uchar>(0, 1) = 0;
  testMat1.at<uchar>(0, 2) = 25;
  testMat1.at<uchar>(0, 3) = 22;
  testMat1.at<uchar>(0, 4) = 17;
  testMat1.at<uchar>(0, 5) = 7;
  testMat1.at<uchar>(0, 6) = 0;
  cout << testMat1 << endl;

  /*
   * Original: [ 10,   0,  25,  22,  17,   7,   0]
   * Expected: [ 10,   0,  25,  22,  \17,  12,   7,   0] with RiInterpolator::IntrIndexPolicy::GreatestGradient
   * Expected: [ 10,   0,  25,  \24,  22,  17,   7,   0] with RiInterpolator::IntrIndexPolicy::LeastGradient
  */
  IntrIndexInfo t1 = riInterpolator.getHorizontalIntrIndex(testMat1, 0, 0, RiInterpolator::IntrIndexPolicy::GreatestGradient, 12);
  t1.print();
  riInterpolator.interpolateHorizontalWindow(testMat1, testMat11, 12, 0, 0, t1);
  cout << testMat11 << endl;

  IntrIndexInfo t2 = riInterpolator.getHorizontalIntrIndex(testMat1, 0, 0, RiInterpolator::IntrIndexPolicy::LeastGradient, 12);
  t2.print();
  riInterpolator.interpolateHorizontalWindow(testMat1, testMat11, 12, 0, 0, t2);
  cout << testMat11 << endl;


  cv::Mat testMat2(1, 7, CV_8UC1, cv::Scalar(0));
  cv::Mat testMat22(1, 8, CV_8UC1, cv::Scalar(0));
  testMat2.at<uchar>(0, 0) = 0;
  testMat2.at<uchar>(0, 1) = 10;
  testMat2.at<uchar>(0, 2) = 0;
  testMat2.at<uchar>(0, 3) = 25;
  testMat2.at<uchar>(0, 4) = 0;
  testMat2.at<uchar>(0, 5) = 14;
  testMat2.at<uchar>(0, 6) = 0;
  cout << testMat2 << endl;

  /*
   * Original: [  0,  10,   0,  25,   0,  14,   0]
   * Expected: [  0,  10,   0,  25,   \0,  0,  14,   0] with RiInterpolator::IntrIndexPolicy::GreatestGradient
   * Expected: [  0,  10,  \0,   0,   25,  0,  14,   0] with RiInterpolator::IntrIndexPolicy::LeastGradient
  */
  IntrIndexInfo t3 = riInterpolator.getHorizontalIntrIndex(testMat2, 0, 0, RiInterpolator::IntrIndexPolicy::GreatestGradient, 30);
  t3.print();
  riInterpolator.interpolateHorizontalWindow(testMat2, testMat22, 5, 0, 0, t3);
  cout << testMat22 << endl;

  IntrIndexInfo t4 = riInterpolator.getHorizontalIntrIndex(testMat2, 0, 0, RiInterpolator::IntrIndexPolicy::LeastGradient, 30);
  t4.print();
  riInterpolator.interpolateHorizontalWindow(testMat2, testMat22, 5, 0, 0, t4);
  cout << testMat22 << endl;

  cv::Mat testMat3(1, 7, CV_8UC1, cv::Scalar(0));
  cv::Mat testMat33(1, 8, CV_8UC1, cv::Scalar(0));
  //IntrIndexInfo t5 = riInterpolator.getHorizontalIntrIndex(testMat3, 0, 0, RiInterpolator::IntrIndexPolicy::LeastGradient, 30);
  IntrIndexInfo t5 = riInterpolator.getHorizontalIntrIndex(testMat3, 0, 0, RiInterpolator::IntrIndexPolicy::GreatestGradient, 30);
  riInterpolator.interpolateHorizontalWindow(testMat3, testMat33, 5, 0, 0, t5);
  cout << testMat3 << endl;
  t5.print();
  cout << testMat33 << endl;

  return 0;
}
