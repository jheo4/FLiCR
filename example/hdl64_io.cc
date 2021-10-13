#include <3dpcc>

using namespace std;

int main() {
  double st, et;

  PcReader pcReader;
  HDL64RIConverter hdl64RiConverter;

  PcWriter pcWriter;


  PclPcXYZI pc;
  PclPcXYZ  pcXyz;

  st = getTsNow();
  pc = pcReader.readXyziBin("0000000000.bin");
  pcXyz = pcReader.readXyzFromXyziBin("0000000000.bin");

  pcl::PolygonMeshPtr mesh = nullptr;
  std:vector<int> partID, pointStates;
  pcReader.generateMeshFromXyz(pcXyz, mesh, partID, pointStates);

  debug_print("points: %d", pc->points.size());
  et = getTsNow();
  debug_print("readT: %f ms", et-st);
  pcWriter.writeBin("pc.bin", pc);
  pcWriter.writePcd("pc_ncomp.pcd", pc, false);
  pcWriter.writePcd("pc_comp.pcd", pc, true);
  pcWriter.writePly("pc.ply", pc);
  pcWriter.writePlyFromMesh("pc_mesh.ply", mesh);

  PclPcXYZI pc2;
  pc2 = pcReader.readXyziBin("pc.bin");
  debug_print("points: %d", pc2->points.size());
  pc2 = pcReader.readXyziPcd("pc_comp.pcd");
  debug_print("points: %d", pc2->points.size());
  pc2 = pcReader.readXyziPcd("pc_ncomp.pcd");
  debug_print("points: %d", pc2->points.size());
  pc2 = pcReader.readXyziPly("pc.ply");
  debug_print("points: %d", pc2->points.size());

  Visualizer pcVisualizer;
  pcVisualizer.initViewerXYZ();
  pcVisualizer.setViewer(*mesh);
  pcVisualizer.show(1000);
}

