#include <3dpcc>

using namespace std;

int main(int argc, char* argv[]) {
  // ./rle_enc orig.bin encoded.bin
  // ./rle_dec orig.bin encoded.bin decoded.bin
  double st, et;

  if(argc != 3) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];

  cout << input << endl;
  cout << output << endl;

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

  uint32_t readBufSize = 4800000;
  char *readBuf = new char[readBufSize];
  uint32_t encodedSize;

  encodedSize = fread(readBuf, sizeof(char), readBufSize, ifp);

  BoostZip boostzip;

  std::vector<char> encoded(readBuf, readBuf + encodedSize);
  std::vector<char> decoded;

  st = getTsNow();
  boostzip.inflateGzip(encoded, decoded);
  et = getTsNow();

  // save file to bin
  fwrite(decoded.data(), sizeof(char), decoded.size(), ofp);
  debug_print("encodedSize: %d, exe: %f", encodedSize, et-st);

  fclose(ifp);
  fclose(ofp);

  return 0;
}

