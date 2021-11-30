#include <3dpcc>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main() {
  double st, et;

  PcReader pcReader;
  HDL64RIConverter hdl64RiConverter;

  PcWriter pcWriter;

  PclPcXYZI pc;
  PclPcXYZ  pcXyz, tmcPcXyz;
  pcl::PolygonMesh mesh;

  st = getTsNow();
  pc = pcReader.readXyziBin("/home/mnt/Data/kitti/0000000000.bin");
  pcXyz    = pcReader.readXyzFromXyziBin("/home/mnt/Data/kitti/0000000000.bin");
  tmcPcXyz = pcReader.readXyzPly("/home/mnt/Data/kitti/tmc3_decomp_pc.ply");

  debug_print("points: %d", pc->points.size());
  et = getTsNow();
  debug_print("readT: %f ms", et-st);

  //pcl::io::loadPLYFile("/home/mnt/Data/kitti/draco_decomp_mesh.ply", mesh);
  //pcl::PointCloud<PclXYZ> meshPc;
  //pcl::fromPCLPointCloud2(mesh.cloud, meshPc);
  PclPcXYZ meshPc = pcReader.readXyzPly("/home/mnt/Data/kitti/draco_decomp_mesh.ply");


  PclMesh t(&mesh);
  //PclPcXYZ tt = static_cast<PclPcXYZ>(&meshPc);
  float PSNR = calcPSNR(pcXyz, meshPc, 80);
  debug_print("Draco PSNR: %f", PSNR);

  int diff = pcXyz->size() - meshPc->size();
  diff = abs(diff);
  float pointError = (float)diff/pcXyz->size();
  debug_print("Draco point errors: %f, diffPoints %d", pointError, diff);


  PSNR = calcPSNR(pcXyz, tmcPcXyz, 80);
  debug_print("TMC3 PSNR: %f", PSNR);
  diff = pcXyz->size() - tmcPcXyz->size();
  diff = abs(diff);
  pointError = (float)diff/pcXyz->size();
  debug_print("TMC3 point errors: %f, diffPoints %d", pointError, diff);

  PclPcXYZ realSpatioXyz = pcReader.readXyzFromXyziBin("/home/mnt/Data/kitti/realtime_spatio_original.bin");
  PclPcXYZ realSpatioDecomXyz = pcReader.readXyzFromXyziBin("/home/mnt/Data/kitti/realtime_spatio_decomp.bin");
  PSNR = calcPSNR(realSpatioXyz, realSpatioDecomXyz, 80);
  debug_print("Realtime-Spatio PSNR: %f", PSNR);
  diff = realSpatioXyz->size() - realSpatioDecomXyz->size();
  diff = abs(diff);
  pointError = (float)diff/realSpatioXyz->size();
  debug_print("Realtime-Spatio point errors: %f, diffPoints %d", pointError, diff);

  Visualizer pcVisualizer;
  pcVisualizer.initViewerXYZ();
  pcVisualizer.setViewer(realSpatioDecomXyz);
  pcVisualizer.show(100000);
}

