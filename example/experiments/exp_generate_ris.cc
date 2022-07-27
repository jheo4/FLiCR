#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv)
{
  cxxopts::Options options("FLiCR", "FLiCR");
  options.add_options()
    ("y, yaml", "YAML file", cxxopts::value<std::string>())
    ("h, help", "Print usage");

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string yamlConfig;
  if(parsedArgs.count("yaml"))
  {
    yamlConfig = parsedArgs["yaml"].as<std::string>();
    std::cout << "YAML Config: " << yamlConfig << std::endl;
  }
  else
  {
    std::cout << "Invalid YAML Config" << std::endl;
    exit(0);
  }

  YAML::Node config = YAML::LoadFile(yamlConfig);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  debug_print("# of scans: %d", numScans);

  std::ostringstream os;
  PcReader pcReader;

  RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
      HDL64_THETA_PRECISION, HDL64_PI_PRECISION_2048,
      HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
      HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

  debug_print("Target RI: %d x %d", riConverter.riCol, riConverter.riRow);

  for(int idx = 0; idx < numScans; idx++)
  {
    // 1. read rawPc
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    debug_print("%s", fn.c_str());
    os.str(""); os.clear();

    types::PclPcXyzi pcXyzi;
    pcXyzi = pcReader.readXyziBin(fn);

    double riMax, riMin, intMax, intMin;
    cv::Mat origRi, normOrigRi;
    cv::Mat intMat, normIntMat;

    riConverter.convertPc2RiWithIm(pcXyzi, origRi, intMat, true);
    riConverter.normalizeRi(origRi, normOrigRi, riMin, riMax);
    riConverter.normalizeRi(intMat, normIntMat, intMin, intMax);

    cv::imwrite(std::string("interpolation/original_")+std::to_string(idx)+std::string(".png"), normOrigRi);

    RiInterpolator riInterpolator;
    cv::Mat normIntrRi, normIntrIntMat;
    cv::Mat normImgRi[5], normImgIntMat[5];
    riInterpolator.interpolate(normOrigRi, normIntMat, normIntrRi, normIntrIntMat, 8, 4, true, 3, RiInterpolator::FARTHEST);


    cv::resize(normOrigRi, normImgRi[0], cv::Size(normIntrRi.cols, normIntrRi.rows), cv::INTER_LINEAR);
    cv::resize(normOrigRi, normImgRi[1], cv::Size(normIntrRi.cols, normIntrRi.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normOrigRi, normImgRi[2], cv::Size(normIntrRi.cols, normIntrRi.rows), 0, 0, cv::INTER_CUBIC);
    cv::resize(normOrigRi, normImgRi[3], cv::Size(normIntrRi.cols, normIntrRi.rows), 0, 0, cv::INTER_LANCZOS4);
    cv::resize(normOrigRi, normImgRi[4], cv::Size(normIntrRi.cols, normIntrRi.rows), 0, 0, cv::INTER_AREA);

    /*
    // To test resized with original
    cv::resize(normIntrRi,   normIntrRi,   cv::Size(normOrigRi.cols, normOrigRi.rows),       cv::INTER_NEAREST);
    cv::resize(normImgRi[0], normImgRi[0], cv::Size(normOrigRi.cols, normOrigRi.rows),       cv::INTER_NEAREST);
    cv::resize(normImgRi[1], normImgRi[1], cv::Size(normOrigRi.cols, normOrigRi.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normImgRi[2], normImgRi[2], cv::Size(normOrigRi.cols, normOrigRi.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normImgRi[3], normImgRi[3], cv::Size(normOrigRi.cols, normOrigRi.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normImgRi[4], normImgRi[4], cv::Size(normOrigRi.cols, normOrigRi.rows), 0, 0, cv::INTER_NEAREST);
    */


    /*
    normImgIntMat[0] = riInterpolator.cvInterpolate(normIntMat, normIntrIntMat.rows, normIntrIntMat.cols, cv::INTER_LINEAR);
    normImgIntMat[1] = riInterpolator.cvInterpolate(normIntMat, normIntrIntMat.rows, normIntrIntMat.cols, cv::INTER_NEAREST);
    normImgIntMat[2] = riInterpolator.cvInterpolate(normIntMat, normIntrIntMat.rows, normIntrIntMat.cols, cv::INTER_CUBIC);
    normImgIntMat[3] = riInterpolator.cvInterpolate(normIntMat, normIntrIntMat.rows, normIntrIntMat.cols, cv::INTER_LANCZOS4);
    normImgIntMat[4] = riInterpolator.cvInterpolate(normIntMat, normIntrIntMat.rows, normIntrIntMat.cols, cv::INTER_AREA);
    */

    cv::imwrite(std::string("interpolation/myinter_")+std::to_string(idx)+std::string(".png"), normIntrRi);
    cv::imwrite(std::string("interpolation/linear_")+std::to_string(idx)+std::string(".png"),  normImgRi[0]);
    cv::imwrite(std::string("interpolation/nearest_")+std::to_string(idx)+std::string(".png"), normImgRi[1]);
    cv::imwrite(std::string("interpolation/cubic_")+std::to_string(idx)+std::string(".png"),   normImgRi[2]);
    cv::imwrite(std::string("interpolation/lanczos_")+std::to_string(idx)+std::string(".png"), normImgRi[3]);
    cv::imwrite(std::string("interpolation/area_")+std::to_string(idx)+std::string(".png"),    normImgRi[4]);

    pcXyzi->clear();

    origRi.release();
    normOrigRi.release();

    normIntrRi.release();
    normIntrIntMat.release();

    for(int i = 0; i < 5; i++)
    {
      normImgRi[i].release();
      normImgIntMat[i].release();
    }

    printProgress((float)idx/(float)numScans);

    os.flush();
    os.clear();
  }
  printProgress(1);

  return 0;
}
