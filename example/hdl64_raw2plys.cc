#include <flicr>

using namespace std;
using namespace flicr;

int main(int argc, char **argv)
{
  if(argc != 4) exit(1);
  std::string sourceDir  = argv[1];
  std::string destDir    = argv[2];
  std::string zeroWidth  = argv[3];

  std::ostringstream os;
  PcReader pcReader;
  PcWriter pcWriter;
  Visualizer pcVisualizer;
  pcVisualizer.initViewerXyz();

  int numScans = countFilesInDirectory(sourceDir.c_str());
  debug_print("# of scans: %d", numScans);

  for(int i = 0; i < numScans; i++)
  {
    os << std::setw(10) << std::setfill('0') << i;
    std::string fn        = sourceDir + "/"    + os.str() + ".bin";
    std::string xyzPlyFn  = destDir + "/xyz/"  + os.str() + ".ply";
    std::string xyziPlyFn = destDir + "/xyzi/" + os.str() + ".ply";
    std::string meshPlyFn = destDir + "/mesh/" + os.str() + ".ply";
    os.str(""); os.clear();

    types::PclPcXyz  pcXyz   = pcReader.readXyzFromXyziBin(fn);
    types::PclPcXyzi pcXyzi  = pcReader.readXyziBin(fn);

    pcl::PolygonMeshPtr mesh = nullptr;

    //mesh = pcReader.readMeshPly(kittiVelodyneMesh + "/" + plyFn);
    std::vector<int> partID, pointStates;
    pcReader.generateMeshFromXyz(pcXyz, mesh, partID, pointStates);

    //pcVisualizer.setViewer(mesh);
    //pcVisualizer.setViewer(pcXyz);
    //pcVisualizer.show(100000);

    pcWriter.writePly(xyzPlyFn, pcXyz); // xyz.ply
    pcWriter.writePly(xyziPlyFn, pcXyzi); // xyzi.ply
    pcWriter.writePlyFromMesh(meshPlyFn, mesh); // mesh.ply

    pcXyz->clear();
    pcXyzi->clear();
    partID.clear();
    pointStates.clear();
    mesh->polygons.clear();
  }

  return 0;
}

