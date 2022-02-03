#include <3dpcc>

using namespace std;

int main(int argc, char* argv[]) {
  // ./qdict_enc orig.bin encoded.bin 4096

  if(argc != 4) exit(1);
  std::string input  = argv[1];
  std::string output = argv[2];
  std::string riRes  = argv[3];

  //cout << input << endl;
  //cout << output << endl;
  //cout << riRes << endl;

  PcReader pcReader;
  RawPc pc;
  FILE* ofp = fopen(output.c_str(), "wb");
  if(ofp == NULL)
  {
    debug_print("ofp exit");
    exit(1);
  }

  pc = pcReader.rawReadXyzBin(input);
  double piPrec = 360.0/stof(riRes);
  HDL64RIConverter converter(HDL64_THETA_PRECISION,
                             piPrec,
                             HDL64_VERTICAL_DEGREE_OFFSET/HDL64_THETA_PRECISION,
                             HDL64_HORIZONTAL_DEGREE_OFFSET/piPrec);

  cv::Mat *ri, nRi;
  double riMin, riMax;
  BoostZip boostZip;
  std::vector<char> dictRes;

  ri = converter.convertRawPc2Ri(pc);
  converter.normalizeRi(ri, &nRi, &riMin, &riMax);
  boostZip.deflateGzip((char*)nRi.data, nRi.elemSize()*nRi.total(), dictRes);

  // save file to bin
  fwrite(dictRes.data(), sizeof(char), dictRes.size(), ofp);

  fclose(ofp);

  return 0;
}

