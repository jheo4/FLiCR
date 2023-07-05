#include "opencv2/imgcodecs.hpp"
#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("utils", "Histogram");
  options.add_options()
    ("tr", "training data directory", cxxopts::value<std::string>())
    ("tr_num", "num of training dataset", cxxopts::value<int>())

    ("tt", "testing data directory", cxxopts::value<std::string>())
    ("tt_num", "num of testing dataset", cxxopts::value<int>())

    ("yaw_fov",   "yaw fov", cxxopts::value<float>())
    ("pitch_fov", "pitch fov", cxxopts::value<float>())
    ("yaw_offset",   "yaw offset", cxxopts::value<float>())
    ("pitch_offset", "pitch offset", cxxopts::value<float>())
    ("min_range", "minimum range", cxxopts::value<float>())
    ("max_range", "maximum range", cxxopts::value<float>())

    ("x, width", "range image width", cxxopts::value<int>())
    ("y, height", "range image height", cxxopts::value<int>())
    ("b, bit", "quantization bits", cxxopts::value<int>())
    ("i, iteration", "training iterations", cxxopts::value<int>())

    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string training_path  = parsedArgs["tr"].as<std::string>();
  int training_data_num = parsedArgs["tr_num"].as<int>();
  std::string testing_path  = parsedArgs["tt"].as<std::string>();
  int testing_data_num = parsedArgs["tt_num"].as<int>();

  float yaw_fov   = parsedArgs["yaw_fov"].as<float>();
  float pitch_fov = parsedArgs["pitch_fov"].as<float>();
  float yaw_offset   = parsedArgs["yaw_offset"].as<float>();
  float pitch_offset = parsedArgs["pitch_offset"].as<float>();
  float min   = parsedArgs["min_range"].as<float>();
  float max   = parsedArgs["max_range"].as<float>();
  int x = parsedArgs["width"].as<int>();
  int y = parsedArgs["height"].as<int>();
  int bit = parsedArgs["bit"].as<int>();
  int iteration = parsedArgs["iteration"].as<int>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\ttraining data path: " << training_path << ", training data num: " << training_data_num << endl;
    cout << "\ttesting data path: " << testing_path << ", testing data num: " << testing_data_num << endl;
    cout << "\tsensor's yaw FoV: "   << yaw_fov << endl;
    cout << "\tsensor's pitch FoV: " << pitch_fov << endl;
    cout << "\tsensor's yaw offset: "   << yaw_offset << endl;
    cout << "\tsensor's pitch offset: " << pitch_offset << endl;
    cout << "\tsensor's min/max range: " << min << ", " << max << endl;
    cout << "\tRI, intMap resolution: " << x << ", " << y << endl;
    cout << "\tQuantization bits: " << bit << endl;
  }

  PcReader pcReader;
  types::PclPcXyzi xyzi = NULL;
  vector<float> points;

  float pitch = pitch_fov / y;
  float yaw   = yaw_fov / x;
  RiConverter riConverter;
  riConverter.setConfig(min, max, pitch, yaw, pitch_fov, yaw_fov, pitch_offset, yaw_offset);

  for (int i = 0; i < training_data_num; i++)
  {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << i;
    std::string file_name = ss.str();

    std::string input = training_path + "/" + file_name + ".bin";
    xyzi = pcReader.readXyziBin(input);
    if (xyzi == NULL)
    {
      if (debug)
        debug_print("reading input file (%s) failed..", input.c_str());
      exit(1);
    }
    cv::Mat ri, intMap;
    riConverter.convertPc2RiWithIm(xyzi, ri, intMap, true);
    for (int i = 0; i < ri.rows; i++)
    {
      for (int j = 0; j < ri.cols; j++)
      {
        float val = ri.at<float>(i, j);
        if (val < min || val > max)
          continue;

        points.push_back(val);
      }
    }

    ri.release();
    intMap.release();
  }

  vector<pair<float, float>> prob;
  vector<pair<float, int>> count;

  for(float point : points)
  {
    bool isExist = false;
    for(auto& c : count)
    {
      if(c.first-0.01 <= point && point <= c.first+0.01) // 5 cm of margin
      {
        c.second += 1;
        isExist = true;
        break;
      }
    }

    if(isExist == false)
    {
      count.push_back(make_pair(point, 1));
    }
  }

  // calculate probability from count
  int total = 0;
  for(auto& c : count)
  {
    total += c.second;
  }

  for(auto& c : count)
  {
    prob.push_back(make_pair(c.first, (float)c.second / total));
  }
  count.clear();


  LloydQuantizer lloyd;
  lloyd.initialize(min, max, bit);
  lloyd.train(points, prob, iteration);

  float avgMSE = 0, avgPSNR = 0;
  // Testing...
  for (int i = 0; i < testing_data_num; i++)
  {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << i;
    std::string file_name = ss.str();

    std::string input = testing_path + "/" + file_name + ".bin";
    xyzi = pcReader.readXyziBin(input);
    if (xyzi == NULL)
    {
      if (debug)
        debug_print("reading input file (%s) failed..", input.c_str());
      exit(1);
    }
    cv::Mat ri, intMap;
    riConverter.convertPc2RiWithIm(xyzi, ri, intMap, true);

    vector<float> testPoints;
    for (int i = 0; i < ri.rows; i++)
    {
      for (int j = 0; j < ri.cols; j++)
      {
        float val = ri.at<float>(i, j);
        if (val < min || val > max)
          continue;

        testPoints.push_back(val);
      }
    }
    ri.release();
    intMap.release();

    float mse, psnr;
    lloyd.test(testPoints, mse, psnr);
    avgMSE += mse;
    avgPSNR += psnr;

    testPoints.clear();
  }

  avgMSE /= testing_data_num;
  avgPSNR /= testing_data_num;
  cout << "[TEST] avgMSE: " << avgMSE << ", avgPSNR: " << avgPSNR << endl;


  return 0;
}

