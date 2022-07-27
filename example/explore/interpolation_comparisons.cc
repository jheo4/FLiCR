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

    double riMax, riMin, intMax, intMin;
    cv::Mat origRi, normOrigRi;
    cv::Mat intMat, normIntMat;
    cv::Mat denormOrigRi, denormIntMat;

    cv::Mat intrNri, dIntrRi;
    cv::Mat intrNintMat;

    riConverter.convertPc2RiWithIm(pcXyzi, origRi, intMat, true); // pcXyzi -> RI & intMat
    riConverter.normalizeRi(origRi, normOrigRi, riMin, riMax);
    riConverter.normalizeRi(intMat, normIntMat, intMin, intMax);

    debug_print("riMin/Max: %f %f", riMin, riMax);
    debug_print("intMin/Max: %f %f", intMin, intMax);

    RiInterpolator riInterpolator;
    double st, et;
    st = getTsNow();
    riInterpolator.interpolate(normOrigRi, normIntMat, intrNri, intrNintMat, 4, 3, true, 3, RiInterpolator::FARTHEST);
    et = getTsNow();
    debug_print("interpolate lat: %f", et-st);

    debug_print("intrNormRi size: %d x %d", intrNri.rows, intrNri.cols);
    cv::imshow("normOrigRi", normOrigRi);
    cv::imshow("normIntMat", normIntMat);

    cv::imshow("intrNri", intrNri);
    cv::imshow("intrNintMat", intrNintMat);

    cv::waitKey();

    cv::imwrite("normOrigRi.jpg", normOrigRi);
    cv::imwrite("normIntMat.jpg", normIntMat);
    cv::imwrite("intrNri.jpg", intrNri);
    cv::imwrite("intrNintMat.jpg", intrNintMat);

    cv::Mat linear;
    cv::Mat nearest;
    cv::Mat cubic;
    cv::Mat lanzos4;
    cv::Mat area;

    cv::Mat linearInt ;
    cv::Mat nearestInt;
    cv::Mat cubicInt  ;
    cv::Mat lanzos4Int;
    cv::Mat areaInt   ;

    cv::resize(normOrigRi, linear , cv::Size(intrNri.cols, intrNri.rows), cv::INTER_LINEAR);
    cv::resize(normOrigRi, nearest, cv::Size(intrNri.cols, intrNri.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normOrigRi, cubic  , cv::Size(intrNri.cols, intrNri.rows), 0, 0, cv::INTER_CUBIC);
    cv::resize(normOrigRi, lanzos4, cv::Size(intrNri.cols, intrNri.rows), 0, 0, cv::INTER_LANCZOS4);
    cv::resize(normOrigRi, area   , cv::Size(intrNri.cols, intrNri.rows), 0, 0, cv::INTER_AREA);

    cv::resize(normIntMat, linearInt , cv::Size(normIntMat.cols, normIntMat.rows), cv::INTER_LINEAR);
    cv::resize(normIntMat, nearestInt, cv::Size(normIntMat.cols, normIntMat.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(normIntMat, cubicInt  , cv::Size(normIntMat.cols, normIntMat.rows), 0, 0, cv::INTER_CUBIC);
    cv::resize(normIntMat, lanzos4Int, cv::Size(normIntMat.cols, normIntMat.rows), 0, 0, cv::INTER_LANCZOS4);
    cv::resize(normIntMat, areaInt   , cv::Size(normIntMat.cols, normIntMat.rows), 0, 0, cv::INTER_AREA);

    cv::Mat dlinear  ;
    cv::Mat dnearest ;
    cv::Mat dcubic   ;
    cv::Mat dlanczos4;
    cv::Mat darea    ;
    cv::Mat dbits    ;

    riConverter.denormalizeRi(normOrigRi, riMin, riMax, denormOrigRi);
    riConverter.denormalizeRi(intrNri, riMin, riMax, dIntrRi);

    riConverter.denormalizeRi(linear,  riMin, riMax, dlinear);
    riConverter.denormalizeRi(nearest, riMin, riMax, dnearest);
    riConverter.denormalizeRi(cubic,   riMin, riMax, dcubic);
    riConverter.denormalizeRi(lanzos4, riMin, riMax, dlanczos4);
    riConverter.denormalizeRi(area,    riMin, riMax, darea);

    RiConverter intrRiConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
        HDL64_THETA_PRECISION, (double)360/intrNri.cols,
        HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
        HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

    types::PclPcXyz xyzOrig    = riConverter.reconstructPcFromRi(denormOrigRi, true);
    types::PclPcXyz xyzIntr    = intrRiConverter.reconstructPcFromRi(dIntrRi, true);
    types::PclPcXyz xyzlinear  = intrRiConverter.reconstructPcFromRi(dlinear, true);
    types::PclPcXyz xyznearest = intrRiConverter.reconstructPcFromRi(dnearest, true);
    types::PclPcXyz xyzcubic = intrRiConverter.reconstructPcFromRi(dcubic, true);
    types::PclPcXyz xyzlanczos = intrRiConverter.reconstructPcFromRi(dlanczos4, true);
    types::PclPcXyz xyzarea = intrRiConverter.reconstructPcFromRi(darea, true);

    visualizer.setViewer(pcXyz);
    visualizer.saveToFile("0raw.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzOrig);
    visualizer.saveToFile("1orig.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzIntr);
    visualizer.saveToFile("2myinter.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzlinear);
    visualizer.saveToFile("3linear.png");
    visualizer.show(3000);

    visualizer.setViewer(xyznearest);
    visualizer.saveToFile("4nearest.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzcubic);
    visualizer.saveToFile("4cubic.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzlanczos);
    visualizer.saveToFile("5lanczos.png");
    visualizer.show(3000);

    visualizer.setViewer(xyzarea);
    visualizer.saveToFile("5area.png");
    visualizer.show(3000);
    pcXyzi->clear();

    origRi.release();
    normOrigRi.release();
    denormOrigRi.release();

    intrNri.release();
    dIntrRi.release();

    printProgress((float)idx/(float)numScans);

    os.flush();
    os.clear();
  }

  printProgress(1);

  return 0;
}

