#include <3dpcc>

using namespace std;

int main(int argc, char* argv[]) {
  // ./qdict_dec orig.bin encoded.bin decoded.bin 4096

  if(argc != 4) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];
  std::string riRes  = argv[3];

  cout << input << endl;
  cout << output << endl;
  cout << riRes << endl;

  PcWriter writer;
  PclPcXYZ pc;

  FILE* ifp = fopen(input.c_str(), "rb");
  if(ifp == NULL)
  {
    debug_print("ifp exit");
    exit(1);
  }

  double piPrec = 360.0/stof(riRes);
  HDL64RIConverter converter(HDL64_THETA_PRECISION,
                             piPrec,
                             HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                             HDL64_HORIZONTAL_DEGREE_OFFSET/piPrec);

  uint32_t readBufSize = 4800000;
  char *readBuf = new char[readBufSize];
  uint32_t encodedSize;
  encodedSize = fread(readBuf, sizeof(char), readBufSize, ifp);

  for(int i = 0; i < 100; i++)
  {
    BoostZip boostzip;

    std::vector<char> encoded(readBuf, readBuf + encodedSize);
    std::vector<char> decoded;

    boostzip.inflateGzip(encoded, decoded);
    cv::Mat recNri(64, stoi(riRes), CV_8UC1, decoded.data());
    cv::Mat recRiFromNri;

    converter.denormalizeRi(&recNri, 80, &recRiFromNri);
    pc = converter.reconstructPcFromRi(&recRiFromNri);

    if(i == 0) writer.writeBin(output, pc);
  }

  fclose(ifp);

  return 0;
}

