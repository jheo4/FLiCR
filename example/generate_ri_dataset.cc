#include <3dpcc>

using namespace std;

int main(int argc, char* argv[]) {
  // ./generate_ri_dataset source_dir dest_dir horizontal_res

  if(argc != 4) exit(1);
  std::string sourceDir  = argv[1];
  std::string destDir    = argv[2];
  std::string horizRes   = argv[3];

  cout << sourceDir << endl;
  cout << destDir << endl;
  cout << horizRes << endl;

  std::ostringstream os;
  PcReader pcReader;
  PcWriter pcWriter;
  float riPrecision = 360.0/stof(horizRes);
  HDL64RIConverter riConverter(HDL64_THETA_PRECISION,
                               riPrecision,
                               HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                               HDL64_HORIZONTAL_DEGREE_OFFSET/riPrecision);
  std::shared_ptr<spdlog::logger> metricLogger = spdlog::basic_logger_st("metLogger", "logs/riDataset/" + horizRes + "_metric.log");
  metricLogger->info("SamplingError\tPSNR\tCD");

  debug_print("RI/IntMap Size: %d x %d", riConverter.riCol, riConverter.riRow);

  int numScans = 0;
  DIR *dir = opendir(sourceDir.c_str());
  if(dir == NULL)
  {
    debug_print("invalide lidarDataPath in config.yaml");
    return 0;
  }
  else
  {
    struct dirent *ent;
    while(ent = readdir(dir))
    {
      if(!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, "..")) {}
      else
      {
        numScans++;
      }
    }
  }
  closedir(dir);

  debug_print("# of scans: %d", numScans);
  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(6) << std::setfill('0') << idx;
    std::string index = os.str();
    std::string fn = sourceDir + "/" + index + ".bin";
    os.str(""); os.clear();

    PclPcXYZI pcXyzi = pcReader.readXyziBin(fn);
    PclPcXYZI rePcXyzi;

    cv::Mat ri, intMap;
    cv::Mat nRi, nIntMap;
    cv::Mat decRi, decIntMap;
    double riMax, riMin, intMax, intMin;

    riConverter.convertPc2RiWithIm(pcXyzi, ri, intMap);

    riConverter.normalizeRi(&ri, &nRi, &riMin, &riMax);
    riConverter.normalizeRi(&intMap, &nIntMap, &intMin, &intMax);

    riConverter.denormalizeRi(&nRi, riMax, &decRi);
    riConverter.denormalizeRi(&nIntMap, intMax, &decIntMap);

    rePcXyzi = riConverter.reconstructPcFromRiWithIm(decRi, decIntMap);

    pcWriter.writeBin(destDir + "/" + index + ".bin", rePcXyzi); 

    // metric logging
    PclPcXYZ origXYZ = xyzi2xyz(pcXyzi);
    PclPcXYZ decXYZ  = xyzi2xyz(rePcXyzi);
    float samplingError = calcSamplingError(origXYZ, decXYZ);
    float PSNR          = calcPSNR(origXYZ, decXYZ, 80);
    float CD            = calcCD(origXYZ, decXYZ);

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

