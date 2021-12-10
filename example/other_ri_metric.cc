#include <3dpcc>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main() {
  double st, et;

  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty())
  {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";

  YAML::Node config = YAML::LoadFile(configYaml);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string metricData    = config["metric_data"].as<std::string>();

  std::ostringstream os;
  PcReader pcReader;

  int numScans = 0;
  DIR *dir = opendir(lidarDataPath.c_str());
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

    PclPcXYZ original  = pcReader.readXyzFromXyziBin(fn);
    PclPcXYZ dracoCl1  = pcReader.readXyzPly(dracoCl1Fn);
    PclPcXYZ dracoCl10 = pcReader.readXyzPly(dracoCl10Fn);
    PclPcXYZ rt2048    = pcReader.readXyzFromXyziBin(rt2048Fn);
    PclPcXYZ rt4096    = pcReader.readXyzFromXyziBin(rt4096Fn);
    PclPcXYZ tmc1cm    = pcReader.readXyzPly(tmc1cmFn);
    PclPcXYZ tmc10cm   = pcReader.readXyzPly(tmc10cmFn);

    float dracoCl1PSNR  = calcPSNR(original, dracoCl1, 80);
    float dracoCl10PSNR = calcPSNR(original, dracoCl10, 80);
    float rt2048PSNR    = calcPSNR(original, rt2048, 80);
    float rt4096PSNR    = calcPSNR(original, rt4096, 80);
    float tmc1cmPSNR    = calcPSNR(original, tmc1cm, 80);
    float tmc10cmPSNR   = calcPSNR(original, tmc10cm, 80);

    float dracoCl1CD    = calcCD(original, dracoCl1);
    float dracoCl10CD   = calcCD(original, dracoCl10);
    float rt2048CD      = calcCD(original, rt2048);
    float rt4096CD      = calcCD(original, rt4096);
    float tmc1cmCD      = calcCD(original, tmc1cm);
    float tmc10cmCD     = calcCD(original, tmc10cm);

    float dracoCl1SE    = calcSamplingError(original, dracoCl1);
    float dracoCl10SE   = calcSamplingError(original, dracoCl10);
    float rt2048SE      = calcSamplingError(original, rt2048);
    float rt4096SE      = calcSamplingError(original, rt4096);
    float tmc1cmSE      = calcSamplingError(original, tmc1cm);
    float tmc10cmSE     = calcSamplingError(original, tmc10cm);

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

