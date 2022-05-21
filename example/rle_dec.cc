#include <flicr>

using namespace std;
using namespace flicr;

// Decode point cloud encoded by RLE
// ./rle_dec orig.bin encoded.bin decoded.bin
int main(int argc, char* argv[]) {
  double st, et;

  if(argc != 4) exit(1);
  std::string orig   = argv[1];
  std::string input  = argv[2];
  std::string output = argv[3];

  cout << orig << endl;
  cout << input << endl;
  cout << output << endl;

  FILE* origfp = fopen(orig.c_str(), "rb");
  if(origfp == NULL)
  {
    debug_print("ifp exit");
    exit(1);
  }
  FILE* ifp = fopen(input.c_str(), "rb");
  if(ifp == NULL)
  {
    debug_print("ifp exit");
    exit(1);
  }
  FILE* ofp = fopen(output.c_str(), "wb");
  if(ofp == NULL)
  {
    debug_print("ofp exit");
    exit(1);
  }

  fseek(origfp, 0L, SEEK_END);
  int origSize = ftell(origfp);

  uint32_t readBufSize = 4800000;
  char *readBuf = new char[readBufSize];
  uint32_t encodedSize;

  encodedSize = fread(readBuf, sizeof(char), readBufSize, ifp);

  RunLengthCompressor rlec;

  std::vector<char> rleRes(readBuf, readBuf + encodedSize);
  st = getTsNow();
  std::vector<char> rlDecoded = rlec.decode(rleRes, origSize);
  et = getTsNow();

  // save file to bin
  fwrite(rlDecoded.data(), sizeof(char), rlDecoded.size(), ofp);
  debug_print("origSize: %d, encodedSize: %d, exe: %f", origSize, encodedSize, et-st);

  fclose(origfp);
  fclose(ifp);
  fclose(ofp);

  return 0;
}

