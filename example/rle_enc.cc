#include <flicr>

using namespace std;
using namespace flicr;

// Encode raw point cloud with RLE
int main(int argc, char* argv[]) {
  cxxopts::Options options("rle_enc", "RLE Encoder");
  options.add_options()
    ("i, input", "Raw input file path", cxxopts::value<std::string>())
    ("o, output", "Encoded output file path", cxxopts::value<std::string>())
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

  std::shared_ptr<spdlog::logger> latencyLogger = spdlog::basic_logger_st("latency_logger", "./rle_enc.log");

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
  float *readBuf = new float[readBufSize];
  uint32_t readCount, size;

  readCount = fread(readBuf, sizeof(float), int(readBufSize/4), ifp);
  size = readCount * 4;

  RunLengthCompressor rlec;
  st = getTsNow();
  std::vector<char> rleRes = rlec.encode((char*)readBuf, size);
  et = getTsNow();

  // save file to bin
  fwrite(rleRes.data(), sizeof(char), rleRes.size(), ofp);

  latencyLogger->info("{}\t{}\t{}", et-st, size, rleRes.size());
  if(debug) debug_print("comp %f, orig size %d, comp size %ld", et-st, size, rleRes.size());


  fclose(ifp);
  fclose(ofp);

  return 0;
}

