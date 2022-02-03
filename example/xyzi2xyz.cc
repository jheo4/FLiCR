#include <3dpcc>

using namespace std;

int main(int argc, char* argv[]) {
  // ./xyzi2xyz xyzi.bin xyz.bin

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

  PcReader reader;
  PcWriter writer;
  PclPcXYZ pc = reader.readXyzFromXyziBin(input);
  writer.writeBin(output, pc);
  fclose(ofp);

  return 0;
}

