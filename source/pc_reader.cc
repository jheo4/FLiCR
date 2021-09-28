#include <pc_reader.h>
#include <utils.h>

PcReader::PcReader()
{
}

PclPcXYZ PcReader::readXyzBin(std::string fileName)
{
  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);

  FILE* fp = fopen(fileName.c_str(), "rb");
  uint32_t readBufSize = 4800000;
  float *readBuf = (float*)malloc(readBufSize);

  uint32_t readCount;
  float *x, *y, *z;

  while(feof(fp) == 0)
  {
    x = readBuf+0;
    y = readBuf+1;
    z = readBuf+2;

    readCount = fread(readBuf, sizeof(float), readBufSize/sizeof(float), fp);
    for(int i = 0; i < int(readCount/3); i++)
    {
      PclXYZ p(std::move(*x), std::move(*y), std::move(*z));
      pc->points.push_back(p);
      x+=3; y+=3; z+=3;
    }
  }

  free(readBuf);
	fclose(fp);

  return pc;
}


PclPcXYZI PcReader::readXyziBin(std::string fileName)
{
  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);

  FILE* fp = fopen(fileName.c_str(), "rb");
  uint32_t readBufSize = 4800000;
  float *readBuf = (float*)malloc(readBufSize);

  uint32_t readCount;
  float *x, *y, *z, *it;

  while(feof(fp) == 0)
  {
    x  = readBuf+0;
    y  = readBuf+1;
    z  = readBuf+2;
    it = readBuf+3;

    readCount = fread(readBuf, sizeof(float), int(readBufSize/4), fp);
    debug_print("ReadCount: %d", readCount);
    for(int i = 0; i < int(readCount/4); i++)
    {
      PclXYZI p;
      p.x         = std::move(*x);
      p.y         = std::move(*y);
      p.z         = std::move(*z);
      p.intensity = std::move(*it);

      pc->points.push_back(p);
      x+=4; y+=4; z+=4; it+=4;
    }
  }

  free(readBuf);
	fclose(fp);

  return pc;
}


PclPcXYZ PcReader::readXyzPcd(std::string fileName)
{
  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);
  pcl::io::loadPCDFile(fileName, *pc);
  return pc;
}


PclPcXYZI PcReader::readXyziPcd(std::string fileName)
{
  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);
  pcl::io::loadPCDFile(fileName, *pc);
  return pc;
}


PclPcXYZ PcReader::readXyzPly(std::string fileName)
{
  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);
  pcl::io::loadPLYFile(fileName, *pc);
  return pc;
}


PclPcXYZI PcReader::readXyziPly(std::string fileName)
{
  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);
  pcl::io::loadPLYFile(fileName, *pc);
  return pc;
}

