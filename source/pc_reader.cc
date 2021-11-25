#include <pc_reader.h>
#include <utils.h>

PcReader::PcReader()
{
}

PclPcXYZ PcReader::readXyzBin(std::string fileName)
{
  FILE* fp = fopen(fileName.c_str(), "rb");
  if(fp == nullptr) return nullptr;

  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);
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


PclPcXYZ PcReader::readXyzFromXyziBin(std::string fileName)
{
  FILE* fp = fopen(fileName.c_str(), "rb");
  if(fp == nullptr) return nullptr;

  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);
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
    //debug_print("ReadCount: %d", readCount/4);
    for(int i = 0; i < int(readCount/4); i++)
    {
      PclXYZ p;
      p.x         = std::move(*x);
      p.y         = std::move(*y);
      p.z         = std::move(*z);

      pc->points.push_back(p);
      x+=4; y+=4; z+=4; it+=4;
    }
  }

  free(readBuf);
	fclose(fp);

  return pc;
}


PclPcXYZI PcReader::readXyziBin(std::string fileName)
{
  FILE* fp = fopen(fileName.c_str(), "rb");
  if(fp == nullptr) return nullptr;

  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);
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
    //debug_print("ReadCount: %d", readCount);
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


bool PcReader::readXyzInt(std::string fileName, PclPcXYZ &pc, std::vector<float> &intensity)
{
  FILE* fp = fopen(fileName.c_str(), "rb");
  if(fp == nullptr) return false;

  pc = PclPcXYZ(new pcl::PointCloud<PclXYZ>);

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
    for(int i = 0; i < int(readCount/4); i++)
    {
      PclXYZ p;
      p.x         = std::move(*x);
      p.y         = std::move(*y);
      p.z         = std::move(*z);
      float itVal = std::move(*it);

      pc->points.push_back(p);
      intensity.push_back(itVal);
      x+=4; y+=4; z+=4; it+=4;
    }
  }

  free(readBuf);
	fclose(fp);

  return true;
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


PclMesh PcReader::readMeshPly(std::string fileName)
{
  PclMesh mesh(new pcl::PolygonMesh);
  pcl::io::loadPLYFile(fileName, *mesh);
  return mesh;
}


void PcReader::generateMeshFromXyz(PclPcXYZ pc, pcl::PolygonMeshPtr &mesh,
                                   std::vector<int> partID, std::vector<int> pointStates)
{
  pcl::NormalEstimation<PclXYZ, pcl::Normal> normEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr norms(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PclXYZ>::Ptr kdTreeXyz(new pcl::search::KdTree<PclXYZ>);

  // Get normal
  kdTreeXyz->setInputCloud(pc);
  normEstimation.setInputCloud(pc);
  normEstimation.setSearchMethod(kdTreeXyz);
  normEstimation.setKSearch(20);
  normEstimation.compute(*norms);

  pcl::PointCloud<pcl::PointNormal>::Ptr pcWithNorm(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*pc, *norms, *pcWithNorm);

  pcl::search::KdTree<pcl::PointNormal>::Ptr kdTreeXyzNorm(new pcl::search::KdTree<pcl::PointNormal>);
  kdTreeXyzNorm->setInputCloud(pcWithNorm);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpTriangulation;
  pcl::PolygonMeshPtr tempMeshes(new pcl::PolygonMesh);
  mesh = tempMeshes;

  gpTriangulation.setSearchRadius(0.5);
  gpTriangulation.setMu(2.5);
  gpTriangulation.setMaximumNearestNeighbors(100);
  gpTriangulation.setMaximumSurfaceAngle(M_PI/4);
  gpTriangulation.setMinimumAngle(M_PI/18);
  gpTriangulation.setMaximumAngle(2*M_PI/3);
  gpTriangulation.setNormalConsistency(false);

  gpTriangulation.setInputCloud(pcWithNorm);
  gpTriangulation.setSearchMethod(kdTreeXyzNorm);
  gpTriangulation.reconstruct(*mesh);


  partID = gpTriangulation.getPartIDs();
  pointStates = gpTriangulation.getPointStates();

  debug_print("polygons: %ld, partIDs %ld, pointStates %ld", mesh->polygons.size(), partID.size(), pointStates.size());
}

