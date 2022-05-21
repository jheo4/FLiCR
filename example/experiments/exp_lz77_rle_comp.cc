#include <flicr>

using namespace std;
using namespace flicr;

int main() {
  FILE* fp = fopen("/home/jin/mnt/Data/kitti/0000000000.bin", "rb");
  uint32_t readBufSize = 4800000;
  float *readBuf = new float[readBufSize];

  uint32_t readCount, size;

  readCount = fread(readBuf, sizeof(float), int(readBufSize/4), fp);
  size = readCount * 4;
  debug_print("size: %d", size);

  double st, et;
  BoostZip boostZip;
  std::vector<char> compressedXyzi;

  st = getTsNow();
  boostZip.deflateGzip((char*)readBuf, size, compressedXyzi);
  et = getTsNow();
  debug_print("Gzip Deflation %f ms", et-st);
  debug_print("Original Data Size %d, Compressed Data Size %ld", size, compressedXyzi.size());

  // inflate nRi1
  std::vector<char> decompressedXyzi;
  st = getTsNow();
  boostZip.inflateGzip(compressedXyzi, decompressedXyzi);
  et = getTsNow();
  debug_print("Gzip Inflation %f ms", et-st);
  debug_print("Compressed Data Size %ld, Decompressed Data Size %ld", compressedXyzi.size(), decompressedXyzi.size());


  RunLengthCompressor rlCompressor;
  st = getTsNow();
  std::vector<char> rlEncoded = rlCompressor.encode((char*)readBuf, size);
  et = getTsNow();
  debug_print("RL encode: %d, %fms", rlEncoded.size(), et-st);
  st = getTsNow();
  std::vector<char> rlDecoded = rlCompressor.decode(rlEncoded, size);
  et = getTsNow();
  debug_print("RL decode: %d, %fms", rlDecoded.size(), et-st);


  delete []readBuf;
  return 0;
}

