#include <flicr>

using namespace std;
using namespace flicr;

// Generate PC dataset with subsampling & quantization
// ./generate_ri_dataset source_dir dest_dir horizontal_res
int main(int argc, char* argv[]) {

  if(argc != 5) exit(1);
  std::string sourceDir  = argv[1];
  std::string destDir    = argv[2];
  std::string horizRes   = argv[3];
  std::string zeroWidth  = argv[4];

  cout << sourceDir << endl;
  cout << destDir << endl;
  cout << horizRes << endl;
  cout << zeroWidth << endl;

  int zeros = stoi(zeroWidth);

  std::ostringstream os;
  PcReader pcReader;
  PcWriter pcWriter;
  float riPrecision = 360.0/stof(horizRes);
  RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                          HDL64_THETA_PRECISION, riPrecision,
                          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/riDataset/" + horizRes + "_metric.log");
  metricLogger->info("SamplingError\tPSNR\tCD");

  debug_print("RI/IntMap Size: %d x %d", riConverter.riCol, riConverter.riRow);

  int numScans = countFilesInDirectory(sourceDir.c_str());
  debug_print("# of scans: %d", numScans);

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(zeros) << std::setfill('0') << idx;
    std::string index = os.str();
    std::string fn = sourceDir + "/" + index + ".bin";
    os.str(""); os.clear();

    types::PclPcXyzi pcXyzi = pcReader.readXyziBin(fn);
    types::PclPcXyzi rePcXyzi;

    cv::Mat ri, intMap;
    cv::Mat nRi, nIntMap;
    cv::Mat decRi, decIntMap;
    double riMax, riMin, intMax, intMin;

    riConverter.convertPc2RiWithIm(pcXyzi, ri, intMap, true);

    riConverter.normalizeRi(ri, nRi, riMin, riMax);
    riConverter.normalizeRi(intMap, nIntMap, intMin, intMax);

    riConverter.denormalizeRi(nRi, riMin, riMax, decRi);
    riConverter.denormalizeRi(nIntMap, intMin, intMax, decIntMap);

    rePcXyzi = riConverter.reconstructPcFromRiWithIm(decRi, decIntMap, true);

    pcWriter.writeBin(destDir + "/" + index + ".bin", rePcXyzi);

    // metric logging
    types::PclPcXyz origXYZ = types::xyzi2xyz(pcXyzi);
    types::PclPcXyz decXYZ  = types::xyzi2xyz(rePcXyzi);
    float samplingError = Metrics::calcPoinNumDiffBtwPcs(origXYZ, decXYZ);
    float PSNR          = Metrics::calcPsnrBtwPcs(origXYZ, decXYZ, HDL64_MAX_RANGE);
    float CD            = Metrics::calcCdBtwPcs(origXYZ, decXYZ);

    metricLogger->info("{}\t{}\t{}", samplingError, PSNR, CD);

    ri.release();
    intMap.release();
    nRi.release();
    nIntMap.release();
    decRi.release();
    decIntMap.release();

    pcXyzi->clear();
    rePcXyzi->clear();
    origXYZ->clear();
    decXYZ->clear();

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}

