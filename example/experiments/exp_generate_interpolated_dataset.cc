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

  std::ostringstream os;
  PcReader pcReader;
  PcWriter pcWriter;
  RiConverter riConverter;
  RiConverter intrRiConverter;

  for(int idx = 0; idx < numScans; idx++)
  {
    int resolutions[4] = {4096, 2048, 1024, 512};

    // subsampled (ri / intensity map)
    cv::Mat ri;
    cv::Mat intMat;

    // interpolated
    cv::Mat intrRi;
    cv::Mat intrIntMat;

    cv::Mat linear;
    cv::Mat nearest;
    cv::Mat cubic;
    cv::Mat lanczos4;
    cv::Mat area;

    cv::Mat intlinear;
    cv::Mat intnearest;
    cv::Mat intcubic;
    cv::Mat intlanczos4;
    cv::Mat intarea;

    // 1. read rawPc
    os << std::setw(10) << std::setfill('0') << idx;
    std::string pcIndex = os.str();
    std::string fn = lidarDataPath + "/" + pcIndex + ".bin";
    debug_print("%s", fn.c_str());
    os.str(""); os.clear();

    types::PclPcXyzi pcXyzi;
    pcXyzi = pcReader.readXyziBin(fn);

    std::string destDir = "interpolation_test/";

    for(int i = 0; i < 4; i++)
    {
      riConverter.setResolution(64, resolutions[i]);
      riConverter.convertPc2RiWithIm(pcXyzi, ri, intMat, true);
      debug_print("Reference RI: %dx%d (%dx%d)", riConverter.riCol, riConverter.riRow, ri.cols, ri.rows);

      RiInterpolator<float> riInterpolator;
      riInterpolator.interpolate(ri, intMat, intrRi, intrIntMat, 4, 2, true, 2, riInterpolator.FARTHEST);

      intrRiConverter.setResolution(64, intrRi.cols);
      debug_print("Interpolated RI: %dx%d (%dx%d)", intrRiConverter.riCol, intrRiConverter.riRow, intrRi.cols, intrRi.rows);

      cv::resize(ri, linear  ,  cv::Size(intrRi.cols, 64), cv::INTER_LINEAR);
      cv::resize(ri, nearest ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_NEAREST);
      cv::resize(ri, cubic   ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_CUBIC);
      cv::resize(ri, lanczos4,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_LANCZOS4);
      cv::resize(ri, area    ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_AREA);

      cv::resize(intMat, intlinear  ,  cv::Size(intrRi.cols, 64), cv::INTER_LINEAR);
      cv::resize(intMat, intnearest ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_NEAREST);
      cv::resize(intMat, intcubic   ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_CUBIC);
      cv::resize(intMat, intlanczos4,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_LANCZOS4);
      cv::resize(intMat, intarea    ,  cv::Size(intrRi.cols, 64), 0, 0, cv::INTER_AREA);

      types::PclPcXyzi xyziOrig    = riConverter.reconstructPcFromRiWithIm    (ri,       intMat, true);
      types::PclPcXyzi xyziIntr    = intrRiConverter.reconstructPcFromRiWithIm(intrRi,   intrIntMat, true);
      types::PclPcXyzi xyzilinear  = intrRiConverter.reconstructPcFromRiWithIm(linear,   intlinear  , true);
      types::PclPcXyzi xyzinearest = intrRiConverter.reconstructPcFromRiWithIm(nearest,  intnearest , true);
      types::PclPcXyzi xyzicubic   = intrRiConverter.reconstructPcFromRiWithIm(cubic,    intcubic   , true);
      types::PclPcXyzi xyzilanczos = intrRiConverter.reconstructPcFromRiWithIm(lanczos4, intlanczos4, true);
      types::PclPcXyzi xyziarea    = intrRiConverter.reconstructPcFromRiWithIm(area,     intarea    , true);

      std::string resolutionDir = destDir + to_string(resolutions[i]) + "/";
      pcWriter.writeBin(resolutionDir + "reference", pcIndex + ".bin", xyziOrig   );
      pcWriter.writeBin(resolutionDir + "myinter"  , pcIndex + ".bin", xyziIntr   );
      pcWriter.writeBin(resolutionDir + "linear"   , pcIndex + ".bin", xyzilinear );
      pcWriter.writeBin(resolutionDir + "nearset"  , pcIndex + ".bin", xyzinearest);
      pcWriter.writeBin(resolutionDir + "cubic"    , pcIndex + ".bin", xyzicubic  );
      pcWriter.writeBin(resolutionDir + "lanczos"  , pcIndex + ".bin", xyzilanczos);
      pcWriter.writeBin(resolutionDir + "area"     , pcIndex + ".bin", xyziarea   );

      /*
      Visualizer visualizer;
      visualizer.initViewerXyz();
      visualizer.setViewer(xyziOrig);
      visualizer.show(500);
      visualizer.setViewer(xyziIntr);
      visualizer.show(500);
      visualizer.setViewer(xyzilinear);
      visualizer.show(500);
      */

      // Clean up...
      ri.release();
      intrRi.release();

      intrRi.release();
      intrIntMat.release();

      linear.release();
      nearest.release();
      cubic.release();
      lanczos4.release();
      area.release();

      intlinear.release();
      intnearest.release();
      intcubic.release();
      intlanczos4.release();
      intarea.release();

      xyziOrig->clear();
      xyziIntr->clear();
      xyzilinear->clear();
      xyzinearest->clear();
      xyzicubic->clear();
      xyzilanczos->clear();
      xyziarea->clear();
    }

    pcXyzi->clear();
    printProgress((float)idx/(float)numScans);

    os.flush();
    os.clear();
  }
  printProgress(1);

  return 0;
}

