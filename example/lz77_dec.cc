#include <flicr>

using namespace std;
using namespace flicr;

// Decode encoded point cloud with LZ77
int main(int argc, char* argv[]) {
  cxxopts::Options options("lz77_enc", "LZ77 Encoder");
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

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./lz77_dec.log");

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

  BoostZip boostzip;

  std::vector<char> encoded(readBuf, readBuf + encodedSize);
  std::vector<char> decoded;

  st = getTsNow();
  boostzip.inflateGzip(encoded, decoded);
  et = getTsNow();

  // save file to bin
  fwrite(decoded.data(), sizeof(char), decoded.size(), ofp);

  latencyLogger->info("{}\t{}\t{}", et-st, encodedSize, decoded.size());
  if(debug) debug_print("lat: %f, encodedSize %d, decodedSize %ld", et-st, encodedSize, decoded.size());

  fclose(ifp);
  fclose(ofp);

  return 0;
}

