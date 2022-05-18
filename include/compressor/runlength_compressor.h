#pragma once

#include <bits/stdc++.h>

namespace flicr
{
class RunLengthCompressor
{
  public:
    std::vector<char> encode(char *data, int dataSize)
    {
      std::vector<char> encoded;

      for(int i = 0; i < dataSize; i++)
      {
        char elem = data[i];
        char elemSeq = 1;
        while(i+1 < dataSize && elemSeq < CHAR_MAX && data[i] == data[i+1])
        {
          elemSeq++;
          i++;
        }
        encoded.push_back(elem);
        encoded.push_back(elemSeq);
      }
      return encoded;
    }


    std::vector<char> decode(std::vector<char> encoded, int origSize)
    {
      std::vector<char> decoded;
      decoded.resize(origSize);

      int decIdx = 0;
      for(int i = 0; i < (int)encoded.size(); i+=2)
      {
        char elem    = encoded[i];
        char elemSeq = encoded[i+1];
        for(int j = 0; j < elemSeq; j++)
        {
          decoded[decIdx + j] = elem;
        }
        decIdx += elemSeq;
      }
      return decoded;
    }
};
}

