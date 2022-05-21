#include <flicr>

using namespace std;
using namespace flicr;

// Encode raw point cloud with LZ77 and RI subsampling
// ./lz77_ri_enc orig.bin encoded.bin 4096
int main(int argc, char* argv[]) {

  if(argc != 4) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];
  std::string riRes  = argv[3];

  cout << input << endl;
  cout << output << endl;
  cout << riRes << endl;

  PcReader pcReader;
  types::PclPcXyz pc;
  FILE* ofp = fopen(output.c_str(), "wb");
  if(ofp == NULL)
  {
    debug_print("ofp exit");
    exit(1);
  }

  pc = pcReader.readXyzFromXyziBin(input);
  double piPrec = 360.0/stof(riRes);
  RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                          HDL64_THETA_PRECISION, piPrec,
                          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

  for(int i = 0; i < 100; i++)
  {
    cv::Mat ri, nRi;
    double riMin, riMax;
    BoostZip boostZip;
    std::vector<char> dictRes;

    riConverter.convertPc2Ri(pc, ri, true);
    riConverter.normalizeRi(ri, nRi, riMin, riMax);
    boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), dictRes);

    // save file to bin
    if(i == 0) fwrite(dictRes.data(), sizeof(char), dictRes.size(), ofp);
  }

  fclose(ofp);

  return 0;
}

