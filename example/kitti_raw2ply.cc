#include <3dpcc>

using namespace std;

int main() {
  double st, et;

  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config = YAML::LoadFile(configYaml);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();
  std::string outputPlyPath = config["ply_path"].as<std::string>();


  std::ostringstream os;
  PcReader pcReader;
  PcWriter pcWriter;

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
  debug_print("# of scans: %d", numScans);


  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn        = lidarDataPath + "/"      + os.str() + ".bin";
    std::string xyzPlyFn  = outputPlyPath + "/xyz_"  + os.str() + ".ply";
    std::string xyziPlyFn = outputPlyPath + "/xyzi_" + os.str() + ".ply";
    os.str(""); os.clear();

    PclPcXYZI pcXyzi;
    PclPcXYZ pcXyz;
    std::vector<float> intensity;
    pcXyz  = pcReader.readXyzFromXyziBin(fn);
    pcXyzi = pcReader.readXyziBin(fn);

    pcWriter.writePly(xyzPlyFn,  pcXyz);
    pcWriter.writePly(xyziPlyFn, pcXyzi);

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}

