#ifndef __PCC_BOOSTZIP__
#define __PCC_BOOSTZIP__

#include <bits/stdc++.h>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

class BoostZip {
  public:
    void deflateGzip(char* data, int size, std::vector<char> &compressed);
    void inflateGzip(std::vector<char> &compressed, std::vector<char> &decompressed);
    std::string deflateStringGzip(const std::string& data);
    std::string inflateStringGzip(const std::string& data);
};

#endif

