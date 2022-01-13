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
  float *readBuf = new float[readBufSize];
  uint32_t readCount, size;

  readCount = fread(readBuf, sizeof(float), int(readBufSize/4), ifp);
  size = readCount * 4;
  debug_print("# of pints: %d, total size: %d", readCount, size);

  RunLengthCompressor rlec;
  std::vector<char> rleRes = rlec.encode((char*)readBuf, size);

  // save file to bin
  fwrite(rleRes.data(), sizeof(char), rleRes.size(), ofp);

  fclose(ifp);
  fclose(ofp);

  return 0;
}

