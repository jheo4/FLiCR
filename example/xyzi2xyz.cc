#include <flicr>

using namespace std;

// Convert point cloud: XYZI.bin to XYZ.bin
// ./xyzi2xyz xyzi.bin xyz.bin
int main(int argc, char* argv[]) {

  if(argc != 3) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];

  cout << input << endl;
  cout << output << endl;

  FILE* ofp = fopen(output.c_str(), "wb");
  if(ofp == NULL)
  {
    debug_print("ofp exit");
    exit(1);
  }

  flicr::PcReader reader;
  flicr::PcWriter writer;
  flicr::types::PclPcXyz pc = reader.readXyzFromXyziBin(input);
  writer.writeBin(output, pc);

  fclose(ofp);

  return 0;
}

