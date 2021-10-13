#include "pcl/io/pcd_io.h"
#include <pc_writer.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils.h>


PcWriter::PcWriter()
{
}


void PcWriter::writeBin(std::string fileName, PclPcXYZ pc)
{
  std::ofstream outStream(fileName, std::ofstream::binary);

  float x, y, z;
  for (auto p : pc->points) {
    x = p.x;
    y = p.y;
    z = p.z;
    outStream.write(reinterpret_cast<const char*>(&x), sizeof(x));
    outStream.write(reinterpret_cast<const char*>(&y), sizeof(y));
    outStream.write(reinterpret_cast<const char*>(&z), sizeof(z));
  }
  outStream.close();
}


void PcWriter::writeBin(std::string fileName, PclPcXYZI pc)
{
  std::ofstream outStream(fileName, std::ofstream::binary);

  float x, y, z, i;
  for (auto p : pc->points) {
    x = p.x;
    y = p.y;
    z = p.z;
    i = p.intensity;
    outStream.write(reinterpret_cast<const char*>(&x), sizeof(x));
    outStream.write(reinterpret_cast<const char*>(&y), sizeof(y));
    outStream.write(reinterpret_cast<const char*>(&z), sizeof(z));
    outStream.write(reinterpret_cast<const char*>(&i), sizeof(i));
  }
  outStream.close();
}


void PcWriter::writePcd(std::string fileName, PclPcXYZ pc, bool compress)
{
  if(compress)
    pcl::io::savePCDFileBinaryCompressed(fileName, *pc);
  else
    pcl::io::savePCDFileBinary(fileName, *pc);
}


void PcWriter::writePcd(std::string fileName, PclPcXYZI pc, bool compress)
{
  if(compress)
    pcl::io::savePCDFileBinaryCompressed(fileName, *pc);
  else
    pcl::io::savePCDFileBinary(fileName, *pc);
}


void PcWriter::writePly(std::string fileName, PclPcXYZ pc)
{
  pcl::io::savePLYFile(fileName, *pc);
}


void PcWriter::writePly(std::string fileName, PclPcXYZI pc)
{
  pcl::io::savePLYFile(fileName, *pc);
}


void PcWriter::writePlyFromMesh(std::string fileName, pcl::PolygonMeshPtr mesh)
{
  pcl::io::savePLYFile(fileName, *mesh);
}

