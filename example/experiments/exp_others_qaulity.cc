#include <flicr>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace flicr;

int main(int argc, char **argv) {
  cxxopts::Options options("FLiCR", "FLiCR");
  options.add_options()
    ("y, yaml", "YAML file", cxxopts::value<std::string>())
    ("h, help", "Print usage")
    ;

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

  double st, et;

  YAML::Node config = YAML::LoadFile(yamlConfig);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string metricData    = config["metric_data"].as<std::string>();

  std::ostringstream os;
  PcReader pcReader;

  int numScans = countFilesInDirectory(lidarDataPath.c_str());
  numScans = 30;
  debug_print("# of scans: %d", numScans);

  double gdracoCl1PSNR {0}, gdracoCl10PSNR {0}, grt2048PSNR {0}, grt4096PSNR {0}, gtmc1cmPSNR {0}, gtmc10cmPSNR {0};
  double gdracoCl1CD {0}, gdracoCl10CD {0}, grt2048CD {0}, grt4096CD {0}, gtmc1cmCD {0}, gtmc10cmCD {0};
  double gdracoCl1SE {0}, gdracoCl10SE {0}, grt2048SE {0}, grt4096SE {0}, gtmc1cmSE {0}, gtmc10cmSE {0};

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    std::string dracoCl1Fn  = metricData + "/draco/xyz/cl1/dec_xyz_" + os.str() + ".ply";
    std::string dracoCl10Fn = metricData + "/draco/xyz/cl10/dec_xyz_" + os.str() + ".ply";
    std::string rt2048Fn    = metricData + "/realtime/2048/" + os.str() + ".bin";
    std::string rt4096Fn    = metricData + "/realtime/4096/" + os.str() + ".bin";
    std::string tmc1cmFn    = metricData + "/tmc13/xyz/1cm/dec_xyz_" + os.str() + ".ply";
    std::string tmc10cmFn   = metricData + "/tmc13/xyz/10cm/dec_xyz_" + os.str() + ".ply";
    os.str(""); os.clear();

    types::PclPcXyz original  = pcReader.readXyzFromXyziBin(fn);
    types::PclPcXyz dracoCl1  = pcReader.readXyzPly(dracoCl1Fn);
    types::PclPcXyz dracoCl10 = pcReader.readXyzPly(dracoCl10Fn);
    types::PclPcXyz rt2048    = pcReader.readXyzFromXyziBin(rt2048Fn);
    types::PclPcXyz rt4096    = pcReader.readXyzFromXyziBin(rt4096Fn);
    types::PclPcXyz tmc1cm    = pcReader.readXyzPly(tmc1cmFn);
    types::PclPcXyz tmc10cm   = pcReader.readXyzPly(tmc10cmFn);

    float dracoCl1PSNR  = Metrics::calcPsnrBtwPcs(original, dracoCl1, 80);
    float dracoCl10PSNR = Metrics::calcPsnrBtwPcs(original, dracoCl10, 80);
    float rt2048PSNR    = Metrics::calcPsnrBtwPcs(original, rt2048, 80);
    float rt4096PSNR    = Metrics::calcPsnrBtwPcs(original, rt4096, 80);
    float tmc1cmPSNR    = Metrics::calcPsnrBtwPcs(original, tmc1cm, 80);
    float tmc10cmPSNR   = Metrics::calcPsnrBtwPcs(original, tmc10cm, 80);

    float dracoCl1CD    = Metrics::calcCdBtwPcs(original, dracoCl1);
    float dracoCl10CD   = Metrics::calcCdBtwPcs(original, dracoCl10);
    float rt2048CD      = Metrics::calcCdBtwPcs(original, rt2048);
    float rt4096CD      = Metrics::calcCdBtwPcs(original, rt4096);
    float tmc1cmCD      = Metrics::calcCdBtwPcs(original, tmc1cm);
    float tmc10cmCD     = Metrics::calcCdBtwPcs(original, tmc10cm);

    float dracoCl1SE    = Metrics::calcPoinNumDiffBtwPcs(original, dracoCl1);
    float dracoCl10SE   = Metrics::calcPoinNumDiffBtwPcs(original, dracoCl10);
    float rt2048SE      = Metrics::calcPoinNumDiffBtwPcs(original, rt2048);
    float rt4096SE      = Metrics::calcPoinNumDiffBtwPcs(original, rt4096);
    float tmc1cmSE      = Metrics::calcPoinNumDiffBtwPcs(original, tmc1cm);
    float tmc10cmSE     = Metrics::calcPoinNumDiffBtwPcs(original, tmc10cm);

    gdracoCl1PSNR  += dracoCl1PSNR;
    gdracoCl10PSNR += dracoCl10PSNR;
    grt2048PSNR    += rt2048PSNR;
    grt4096PSNR    += rt4096PSNR;
    gtmc1cmPSNR    += tmc1cmPSNR;
    gtmc10cmPSNR   += tmc10cmPSNR;

    gdracoCl1CD    += dracoCl1CD;
    gdracoCl10CD   += dracoCl10CD;
    grt2048CD      += rt2048CD;
    grt4096CD      += rt4096CD;
    gtmc1cmCD      += tmc1cmCD;
    gtmc10cmCD     += tmc10cmCD;

    gdracoCl1SE    += dracoCl1SE;
    gdracoCl10SE   += dracoCl10SE;
    grt2048SE      += rt2048SE;
    grt4096SE      += rt4096SE;
    gtmc1cmSE      += tmc1cmSE;
    gtmc10cmSE     += tmc10cmSE;
  }

  gdracoCl1PSNR  /= numScans;
  gdracoCl10PSNR /= numScans;
  grt2048PSNR    /= numScans;
  grt4096PSNR    /= numScans;
  gtmc1cmPSNR    /= numScans;
  gtmc10cmPSNR   /= numScans;

  gdracoCl1CD    /= numScans;
  gdracoCl10CD   /= numScans;
  grt2048CD      /= numScans;
  grt4096CD      /= numScans;
  gtmc1cmCD      /= numScans;
  gtmc10cmCD     /= numScans;

  gdracoCl1SE    /= numScans;
  gdracoCl10SE   /= numScans;
  grt2048SE      /= numScans;
  grt4096SE      /= numScans;
  gtmc1cmSE      /= numScans;
  gtmc10cmSE     /= numScans;

  printf("      DracoCl1, DracoCl10, rt2048, rt4096, tmc1cm, tmc10cm\n");
  printf("PSNR: %f, %f, %f, %f, %f, %f\n", gdracoCl1PSNR, gdracoCl10PSNR, grt2048PSNR, grt4096PSNR, gtmc1cmPSNR, gtmc10cmPSNR);
  printf("CD:   %f, %f, %f, %f, %f, %f\n", gdracoCl1CD, gdracoCl10CD, grt2048CD, grt4096CD, gtmc1cmCD, gtmc10cmCD);
  printf("SE:   %f, %f, %f, %f, %f, %f\n", gdracoCl1SE, gdracoCl10SE, grt2048SE, grt4096SE, gtmc1cmSE, gtmc10cmSE);
}

