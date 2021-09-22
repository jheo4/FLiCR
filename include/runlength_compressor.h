#ifndef __PCC_RUNLENGTH_COMPRESSOR__
#define __PCC_RUNLENGTH_COMPRESSOR__

#include <bits/stdc++.h>

class RunLengthCompressor {
  public:
    std::vector<char> encode(char *data, int dataSize);
    std::vector<char> decode(std::vector<char> encoded, int origSize);
};

#endif

