#include <3dpcc>
#include <bits/stdc++.h>

using namespace std;

int main() {
  // for profiling
  double st, et;

  PcReader pcReader;

  PclPcXYZ pc = pcReader.readXyzPly("/home/jin/mnt/github/PCC_Comp/mpeg-pcc-tmc13/test/reconstructed.ply");

  Visualizer visualizer;
  visualizer.initViewerXYZ();

  /* PC Visualization */
  visualizer.setViewer(pc);
  for(int i = 0; i < 1; i++) {
    visualizer.show(5000);
  }

  return 0;
}

