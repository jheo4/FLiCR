#include <tiler.h>


std::vector<cv::Mat> Tiler::split(cv::Mat image, int x_tiles, int y_tiles)
{
  std::vector<cv::Mat> tiles;
  int tileWidth = image.cols / x_tiles;
  int tileHeight = image.rows / y_tiles;

  debug_print("tileWidth: %d, tileHeight: %d\n", tileWidth, tileHeight);

  for (int i = 0; i < y_tiles; i++)
  {
    for (int j = 0; j < x_tiles; j++)
    {
      cv::Rect roi(j * tileWidth, i * tileHeight, tileWidth, tileHeight);
      cv::Mat tile = image(roi);
      tiles.push_back(tile);
    }
  }
  return tiles;
}
