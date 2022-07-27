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

    types::PclPcXyz pcXyz;
    std::vector<float> intensity;

    if(pcReader.readXyzInt(fn, pcXyz, intensity) == false)
    {
      debug_print("pcReader.readXyzInt failed...");
      return -1;
    }

    double riMax, riMin;
    cv::Mat origRi, normOrigRi;
    cv::Mat denormOrigRi;
    types::PclPcXyz recXyz;

    cv::Mat normIntrRi, denormIntrRi;
    types::PclPcXyz recIntrXyz;

    // cv::Mat compNri, compRi, compDeRi; // original --> 4096 nri --> 4096 ri
    // types::PclPcXyz compXyz;

    // 2. PC --> origRi --> normOrigRi
    riConverter.convertPc2Ri(pcXyz, origRi, true);
    riConverter.normalizeRi(origRi, normOrigRi, riMin, riMax);
    riConverter.denormalizeRi(normOrigRi, riMin, riMax, denormOrigRi);
    recXyz = riConverter.reconstructPcFromRi(denormOrigRi, true);


    RiInterpolator riInterpolator;
    double st, et;
    st = getTsNow();
    riInterpolator.interpolate(normOrigRi, normIntrRi, 4, 3, true, 3, RiInterpolator::NEAREST);
    et = getTsNow();
    debug_print("interpolate lat: %f", et-st);

    debug_print("intrNormRi size: %d x %d", normIntrRi.rows, normIntrRi.cols);
    cv::imshow("normOrigRi", normOrigRi);
    cv::imshow("intrNormRi", normIntrRi);
    cv::waitKey();

    RiConverter intrRiConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
        HDL64_THETA_PRECISION, (double)360/normIntrRi.cols,
        HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
        HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
    intrRiConverter.denormalizeRi(normIntrRi, riMin, riMax, denormIntrRi);
    recIntrXyz = intrRiConverter.reconstructPcFromRi(denormIntrRi, true);


    visualizer.setViewer(pcXyz);
    visualizer.saveToFile("1Orig_File.png");
    visualizer.show(5000);

    visualizer.setViewer(recIntrXyz);
    visualizer.saveToFile("2Intr_File.png");
    visualizer.show(5000);

    visualizer.setViewer(recXyz);
    visualizer.saveToFile("3Subsampled_File.png");
    visualizer.show(5000);


    /*

      // 2. pc --> comp Ri --> comp nRi
      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, HDL64_PI_PRECISION_1024,
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

      riConverter.convertPc2Ri(pcXyz, compRi, true);
      riConverter.normalizeRi(compRi, compNri, riMin, riMax);
      riConverter.denormalizeRi(compNri, riMin, riMax, compDeRi);
      compXyz = riConverter.reconstructPcFromRi(compDeRi, true);

      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, HDL64_PI_PRECISION_512,
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);


      // side-channel: pc --> ri --> nri --> deRi --> recPc
      riConverter.denormalizeRi(nRi, riMin, riMax, deRi);
      recXyz = riConverter.reconstructPcFromRi(deRi, true);

      // 3. interpolate
      RiInterpolator riInterpolator;
      riInterpolator.setIntr(riConverter.riRow, riConverter.riCol, INTR_WIND_SIZE, 1); // origRow, origCol, hWnd, vWnd
      riInterpolator.printSetting();
      intrNri = riInterpolator.interpolateHorizontal(nRi, RiInterpolator::GreatestGradient, 10);

      // 4. riConverter reset -- intrCol, intrRow
      riConverter.setConfig(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                            HDL64_THETA_PRECISION, HDL64_HORIZONTAL_DEGREE/(double)riInterpolator.intrCol,
                            HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                            HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);
      debug_print("Intr RI: %d x %d", riConverter.riCol, riConverter.riRow);

      // 3. intrNri --> intrRi --> intrPc
      riConverter.denormalizeRi(intrNri, riMin, riMax, intrDeRi);
      intrXyz = riConverter.reconstructPcFromRi(intrDeRi, true);

      debug_print("Orig # of points: %ld", pcXyz->size());
      debug_print("Target # of points: %ld", recXyz->size());
      debug_print("Intr # of points: %ld", intrXyz->size());

      //cv::imshow("orig nRI", nRi);
      cv::imshow("compNri", compNri);
      cv::imshow("targetNrI", nRi);
      cv::imshow("intrNrI", intrNri);
      cv::waitKey();
      */


      /*
      visualizer.setViewer(pcXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("1Orig_File.png");
      visualizer.show(1000);
      */

    /*
      visualizer.setViewer(recXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("1_target.png");
      visualizer.show(5000);

      visualizer.setViewer(compXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("2_comp.png");
      visualizer.show(5000);

      visualizer.setViewer(intrXyz);
      visualizer.setViewerBEV(70);
      visualizer.saveToFile("3_intr.png");
      visualizer.show(5000);
      */

    pcXyz->clear();
    intensity.clear();

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
