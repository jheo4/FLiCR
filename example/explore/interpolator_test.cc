#include <flicr>

using namespace std;
using namespace flicr;

int main()
{
  RiInterpolator<uint8_t> riInterpolator;
  riInterpolator.setIntr(64, 4500, 2, 2);
  riInterpolator.printSetting();

  cv::Mat testMat(1, 8, CV_8UC1, cv::Scalar(0));
  testMat.at<uchar>(0, 0) = 0 ;
  testMat.at<uchar>(0, 1) = 0 ;
  testMat.at<uchar>(0, 2) = 3 ;
  testMat.at<uchar>(0, 3) = 5 ;
  testMat.at<uchar>(0, 4) = 9 ;
  testMat.at<uchar>(0, 5) = 0 ;
  testMat.at<uchar>(0, 6) = 4 ;
  testMat.at<uchar>(0, 7) = 6 ;
  cout << testMat << endl;

  cv::Mat intrMat;
  riInterpolator.interpolate(testMat, intrMat, 4, 2, true, 4);
  cout << intrMat << endl;


  int test;
  test = riInterpolator.getRiX(-1, 360, 0, true);
  cout << test << endl; // 359

  test = riInterpolator.getRiX(-1, 360, 0, false);
  cout << test << endl; // -1

  test = riInterpolator.getRiX(0, 360, -1, true);
  cout << test << endl; // 359
  test = riInterpolator.getRiX(0, 360, -2, true);
  cout << test << endl; // 358

  test = riInterpolator.getRiX(365, 360, 0, true);
  cout << test << endl; // 5
  test = riInterpolator.getRiX(361, 360, 0, false);
  cout << test << endl; // -1

  test = riInterpolator.getRiX(359, 360, 1, true);
  cout << test << endl; // 0
  test = riInterpolator.getRiX(359, 360, 5, true);
  cout << test << endl; // 4
  test = riInterpolator.getRiX(359, 360, 1, false);
  cout << test << endl; // -1
  return 0;
}
