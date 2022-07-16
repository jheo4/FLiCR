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
  numScans = 1;

  std::ostringstream os;
  PcReader pcReader;
  Visualizer visualizer;
  visualizer.initViewerXyz();

  RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
      HDL64_THETA_PRECISION, HDL64_PI_PRECISION_1024,
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
    cv::Mat denormOrigRi, denormIntMat;

    types::PclPcXyz recXyz;

    cv::Mat normIntrRi, denormIntrRi;
    cv::Mat normIntrIntMat;
    types::PclPcXyz recIntrXyz;

    // 2. PC --> origRi --> normOrigRi
    riConverter.convertPc2RiWithIm(pcXyzi, origRi, intMat, true);

    riConverter.normalizeRi(origRi, normOrigRi, riMin, riMax);
    riConverter.normalizeRi(intMat, normIntMat, intMin, intMax);

    RiInterpolator riInterpolator;
    double st, et;
    st = getTsNow();
    riInterpolator.interpolate(normOrigRi, normIntMat, normIntrRi, normIntrIntMat, 4, 3, true, 3, RiInterpolator::FARTHEST);
    et = getTsNow();
    debug_print("interpolate lat: %f", et-st);

    debug_print("intrNormRi size: %d x %d", normIntrRi.rows, normIntrRi.cols);
    cv::imshow("normOrigRi", normOrigRi);
    cv::imshow("normIntMat", normIntMat);
    cv::imshow("intrNormRi", normIntrRi);
    cv::imshow("normIntrIntMat", normIntrIntMat);
    cv::waitKey();

    /*
    RiConverter intrRiConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
        HDL64_THETA_PRECISION, (double)360/normIntrRi.cols,
        HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
        HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
    intrRiConverter.denormalizeRi(normIntrRi, riMin, riMax, denormIntrRi);
    recIntrXyz = intrRiConverter.reconstructPcFromRi(denormIntrRi, true);

    visualizer.setViewer(recIntrXyz);
    visualizer.saveToFile("2Intr_File.png");
    visualizer.show(5000);

    visualizer.setViewer(recXyz);
    visualizer.saveToFile("3Subsampled_File.png");
    visualizer.show(5000);
    */

    pcXyzi->clear();

    origRi.release();
    normOrigRi.release();
    denormOrigRi.release();

    normIntrRi.release();
    denormIntrRi.release();

    printProgress((float)idx/(float)numScans);

    os.flush();
    os.clear();
  }

  printProgress(1);

  return 0;
}

