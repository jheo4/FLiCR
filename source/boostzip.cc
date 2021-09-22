#include "boostzip.h"

// Gzip_compressor: https://www.boost.org/doc/libs/1_62_0/libs/iostreams/doc/classes/gzip.html
// Zlib details: https://en.wikipedia.org/wiki/Zlib
// Zlib deflation algo: https://en.wikipedia.org/wiki/LZ77_and_LZ78#LZ77
// Zlib deflation algo: https://en.wikipedia.org/wiki/Huffman_coding
void BoostZip::deflateGzip(char* data, int size, std::vector<char> &compressed)
{
  namespace bio = boost::iostreams;

  bio::filtering_ostreambuf out;
  //out.push(bio::gzip_compressor(bio::gzip_params(bio::gzip::best_compression)));
  //out.push(bio::gzip_compressor(bio::gzip_params(bio::gzip::best_speed)));
  out.push(bio::gzip_compressor(bio::gzip_params(1)));
  out.push(bio::back_inserter(compressed));
  bio::write(out, data, size);
}


void BoostZip::inflateGzip(std::vector<char> &compressed, std::vector<char> &decompressed)
{
  namespace bio = boost::iostreams;

  bio::filtering_ostreambuf out;
  out.push(bio::gzip_decompressor());
  out.push(bio::back_inserter(decompressed));
  bio::write(out, &compressed[0], compressed.size());
}


std::string BoostZip::deflateStringGzip(const std::string& data)
{
  namespace bio = boost::iostreams;

  std::stringstream compressed;
  std::stringstream origin(data);

  bio::filtering_streambuf<bio::input> out;
  out.push(bio::gzip_compressor(bio::gzip_params(bio::gzip::best_compression)));
  out.push(origin);
  bio::copy(out, compressed);

  return compressed.str();
}


std::string BoostZip::inflateStringGzip(const std::string& data)
{
  namespace bio = boost::iostreams;

  std::stringstream compressed(data);
  std::stringstream decompressed;

  bio::filtering_streambuf<bio::input> out;
  out.push(bio::gzip_decompressor());
  out.push(compressed);
  bio::copy(out, decompressed);

  return decompressed.str();
}

