#include <flicr>

using namespace std;
using namespace flicr;

// Decode point cloud encoded by RLE
int main(int argc, char* argv[]) {
  cxxopts::Options options("rle_dec", "RLE Decoder");
  options.add_options()
    ("i, input", "Encoded input file path", cxxopts::value<std::string>())
    ("o, output", "Decoded output file path", cxxopts::value<std::string>())
    ("d, debug", "debug print option", cxxopts::value<bool>()->default_value("false"))
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string input  = parsedArgs["input"].as<std::string>();
  std::string output = parsedArgs["output"].as<std::string>();
  bool debug = parsedArgs["debug"].as<bool>();

  if(debug)
  {
    cout << "ARGS" << endl;
    cout << "\tinput file: " << input << endl;
    cout << "\toutput file: " << output << endl;
  }

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./rle_dec.log");

  double st, et;

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

  RunLengthCompressor rlec;

  std::vector<char> rleRes(readBuf, readBuf + encodedSize);
  st = getTsNow();
  std::vector<char> rlDecoded = rlec.decode(rleRes);
  et = getTsNow();

  // save file to bin
  fwrite(rlDecoded.data(), sizeof(char), rlDecoded.size(), ofp);

  latencyLogger->info("{}\t{}\t{}", et-st, encodedSize, rlDecoded.size());
  if(debug) debug_print("decomp %f, encodedsize %d, decodedSize %ld", et-st, encodedSize, rlDecoded.size());

  fclose(ifp);
  fclose(ofp);

  return 0;
}

