#include <3dpcc>

using namespace std;

int main() {
  double st, et;

  /* Set configs from yaml */
  std::string pccHome = getenv("PCC_HOME");
  if(pccHome.empty()) {
    std::cout << "set PCC_HOME" << std::endl;
    return 0;
  }
  std::string configYaml = pccHome + "/config.yaml";
  std::cout << "3D PCC config.yaml: " << configYaml << std::endl;

  YAML::Node config = YAML::LoadFile(configYaml);

  std::string kittiVelodyneRaw  = config["kitti_raw_lidar"].as<std::string>();
  std::string kittiVelodyneXyz  = config["kitti_xyz_ply"].as<std::string>();
  std::string kittiVelodyneXyzi = config["kitti_xyzi_ply"].as<std::string>();
  std::string kittiVelodyneMesh = config["kitti_mesh_ply"].as<std::string>();

  PcReader pcReader;
  PcWriter pcWriter;
  Visualizer pcVisualizer;
  pcVisualizer.initViewerXYZ();

  for(int i = 0; i < 373; i++)
  {
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << i;
    std::string fIdx = ss.str();
    std::string inFn = fIdx + ".bin";
    std::string plyFn = fIdx + ".ply";
    std::string outFn;

    PclPcXYZ pcXyz   = pcReader.readXyzFromXyziBin(kittiVelodyneRaw + "/" + inFn);
    //PclPcXYZI pcXyzi = pcReader.readXyziBin(kittiVelodyneRaw + "/" + inFn);

    pcl::PolygonMeshPtr mesh = nullptr;
    outFn = fIdx + ".ply";

    //mesh = pcReader.readMeshPly(kittiVelodyneMesh + "/" + plyFn);
    std::vector<int> partID, pointStates;
    st = getTsNow();
    pcReader.generateMeshFromXyz(pcXyz, mesh, partID, pointStates);
    et = getTsNow();
    debug_print("mesh generation tme: %f", et-st);

    //pcVisualizer.setViewer(mesh);
    //pcVisualizer.setViewer(pcXyz);
    //pcVisualizer.show(100000);

    //pcWriter.writePly(outFn, pcXyz); // xyz.ply
    //pcWriter.writePly(outFn, pcXyzi); // xyzi.ply
    //pcWriter.writePlyFromMesh(outFn, mesh); // mesh.ply

    //pcXyz->clear();
    //partID.clear(); pointStates.clear();
    mesh->polygons.clear();
    //pcXyzi->clear();
  }

  return 0;
}

