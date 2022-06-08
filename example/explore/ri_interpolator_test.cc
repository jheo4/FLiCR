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

  double piPrec[6] = {HDL64_PI_PRECISION_4500, HDL64_PI_PRECISION_4096, HDL64_PI_PRECISION_2048,
                      HDL64_PI_PRECISION_1024, HDL64_PI_PRECISION_512, HDL64_PI_PRECISION_256};

  std::ostringstream os;
  PcReader pcReader;
  Visualizer visualizer;
  visualizer.initViewerXyz();

  for(int prec = 3; prec < 4; prec++)
  {
    RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, piPrec[prec],
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
    debug_print("%d x %d", riConverter.riCol, riConverter.riRow);

    for(int idx = 0; idx < numScans; idx++)
    {
      // 1. read rawPc
      os << std::setw(10) << std::setfill('0') << idx;
      std::string fn = lidarDataPath + "/" + os.str() + ".bin";
      debug_print("%s", fn.c_str());
      os.str(""); os.clear();

      types::PclPcXyz pcXyz;
      std::vector<float> intensity;

      if(pcReader.readXyzInt(fn, pcXyz, intensity) == false) return -1;

      cv::Mat ri, nRi; // original --> nri
      cv::Mat deRi;    // original --> nri --> deRi
      cv::Mat compNri, compRi, compRi2; // original --> 4096 nri --> 4096 ri
      cv::Mat intrNri, intrRi; // original --> intrNri  --> intrRi
      double riMax, riMin;
      types::PclPcXyz intrXyz, recXyz, compXyz;

      // 2. pc --> ri --> nri
      riConverter.convertPc2Ri(pcXyz, ri, true);
      riConverter.normalizeRi(ri, nRi, riMin, riMax);

      // 2. pc --> 4096ri --> 4096nri
      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
          HDL64_THETA_PRECISION, HDL64_PI_PRECISION_4096,
          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
      riConverter.convertPc2Ri(pcXyz, compRi, true);
      riConverter.normalizeRi(compRi, compNri, riMin, riMax);
      riConverter.denormalizeRi(compNri, riMin, riMax, compRi2);
      compXyz = riConverter.reconstructPcFromRi(compRi2, true);

      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
          HDL64_THETA_PRECISION, piPrec[prec],
          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);


      // side-channel: pc --> ri --> nri --> deRi --> recPc
      riConverter.denormalizeRi(nRi, riMin, riMax, deRi);
      recXyz = riConverter.reconstructPcFromRi(deRi, true);

      // 3. interpolate
      RiInterpolator riInterpolator;
      riInterpolator.setIntr(riConverter.riRow, riConverter.riCol, 3, 1); // origRow, origCol, hWnd, vWnd
      riInterpolator.printSetting();
      intrNri = riInterpolator.interpolateHorizontal(nRi, RiInterpolator::GreatestGradient, 10);

      // 4. riConverter reset -- intrCol, intrRow
      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
          HDL64_THETA_PRECISION, HDL64_HORIZONTAL_DEGREE/(double)riInterpolator.intrCol,
          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
      debug_print("%d x %d", riConverter.riCol, riConverter.riRow);

      // 3. intrNri --> intrRi --> intrPc
      riConverter.denormalizeRi(intrNri, riMin, riMax, intrRi);
      intrXyz = riConverter.reconstructPcFromRi(intrRi, true);

      debug_print("Orig # of points: %ld", pcXyz->size());
      debug_print("Rec  # of points: %ld", recXyz->size());
      debug_print("Intr # of points: %ld", intrXyz->size());

      cv::imshow("nRI", nRi);
      cv::imshow("4096nRi", compNri);
      cv::imshow("intrNrI", intrNri);
      cv::waitKey();


      visualizer.setViewer(pcXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("1Orig_File.png");
      visualizer.show(1000);

      visualizer.setViewer(recXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("2Rec_File.png");
      visualizer.show(1000);

      visualizer.setViewer(compXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("3-4096_File.png");
      visualizer.show(1000);

      visualizer.setViewer(intrXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("4Intr_File.png");
      visualizer.show(1000);

      nRi.release();
      ri.release();
      nRi.release();
      printProgress((float)idx/(float)numScans);

      pcXyz->clear();
      intensity.clear();
    }
    printProgress(1);

    os.flush();
    os.clear();
  }

  return 0;
}

