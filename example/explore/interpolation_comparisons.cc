#include <flicr>

#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

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
    types::PclPcXyz pcXyz;
    pcXyzi = pcReader.readXyziBin(fn);
    pcXyz = pcReader.readXyzFromXyziBin(fn);

    cv::Mat origRi;
    cv::Mat intMat;

    cv::Mat intrRi;
    cv::Mat intrIntMat;

    riConverter.convertPc2RiWithIm(pcXyzi, origRi, intMat, true); // pcXyzi -> RI & intMat

    RiInterpolator<float> riInterpolator;
    double st, et;
    st = getTsNow();
    riInterpolator.interpolate(origRi, intMat, intrRi, intrIntMat, 8, 4, true, 2, riInterpolator.FARTHEST);
    et = getTsNow();
    debug_print("interpolate lat: %f", et-st);
    debug_print("interpolated size: %dx%d --> %dx%d", origRi.cols, origRi.rows, intrRi.cols, intrRi.rows);

    cv::Mat linear;
    cv::Mat nearest;
    cv::Mat cubic;
    cv::Mat lanczos4;
    cv::Mat area;

    cv::Mat linearInt ;
    cv::Mat nearestInt;
    cv::Mat cubicInt  ;
    cv::Mat lanzos4Int;
    cv::Mat areaInt   ;

    cv::resize(origRi, linear ,  cv::Size(intrRi.cols, 64), cv::INTER_LINEAR);
    st = getTsNow();
    cv::resize(origRi, nearest,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_NEAREST);
    et = getTsNow();
    debug_print("nearest interpolate lat: %f", et-st);
    cv::resize(origRi, cubic  ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_CUBIC);
    cv::resize(origRi, lanczos4, cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_LANCZOS4);
    cv::resize(origRi, area   ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_AREA);

    RiConverter intrRiConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
        HDL64_THETA_PRECISION, (double)360/intrRi.cols,
        HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
        HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

    types::PclPcXyz xyzOrig    = riConverter.reconstructPcFromRi(origRi, true);
    types::PclPcXyz xyzIntr    = intrRiConverter.reconstructPcFromRi(intrRi, true);
    types::PclPcXyz xyzlinear  = intrRiConverter.reconstructPcFromRi(linear, true);
    types::PclPcXyz xyznearest = intrRiConverter.reconstructPcFromRi(nearest, true);
    types::PclPcXyz xyzcubic   = intrRiConverter.reconstructPcFromRi(cubic, true);
    types::PclPcXyz xyzlanczos = intrRiConverter.reconstructPcFromRi(lanczos4, true);
    types::PclPcXyz xyzarea    = intrRiConverter.reconstructPcFromRi(area, true);

    visualizer.setViewer(pcXyz);
    visualizer.saveToFile("0raw.png");
    visualizer.show(300);

    visualizer.setViewer(xyzOrig);
    visualizer.saveToFile("1orig.png");
    visualizer.show(300);

    visualizer.setViewer(xyzIntr);
    visualizer.saveToFile("2intr.png");
    visualizer.show(300);

    visualizer.setViewer(xyzlinear);
    visualizer.saveToFile("3linear.png");
    visualizer.show(300);

    visualizer.setViewer(xyznearest);
    visualizer.saveToFile("4nearest.png");
    visualizer.show(300);

    visualizer.setViewer(xyzcubic);
    visualizer.saveToFile("4cubic.png");
    visualizer.show(300);

    visualizer.setViewer(xyzlanczos);
    visualizer.saveToFile("5lanczos.png");
    visualizer.show(300);

    visualizer.setViewer(xyzarea);
    visualizer.saveToFile("5area.png");
    visualizer.show(300);
    pcXyzi->clear();

    origRi.release();
    printProgress((float)idx/(float)numScans);

    os.flush();
    os.clear();
  }

  printProgress(1);

  return 0;
}

