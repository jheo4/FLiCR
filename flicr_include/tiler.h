#pragma once

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <defs.h>

class Tiler
{
public:
  std::vector<cv::Mat> split(cv::Mat image, int x_tiles, int y_tiles);
  cv::Mat merge(std::vector<cv::Mat> tiles, int x_tiles, int y_tiles);
};
