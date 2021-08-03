#include <hdl64_pc_reader.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils.h>


HDL64PCReader::HDL64PCReader(): BagReader()
{
  view = nullptr;
}


HDL64PCReader::HDL64PCReader(std::string bagFile, std::string topic): BagReader(bagFile, topic)
{
  view = nullptr;
  openBag(bagFile, topic);
}


PclPcXYZ HDL64PCReader::getNextPC()
{
  double st = getTsNow();
  sensor_msgs::PointCloud2::ConstPtr velodynePointCloud = curMsg->instantiate<sensor_msgs::PointCloud2>();

  PclPcXYZ pc(new pcl::PointCloud<PclXYZ>);

  if(velodynePointCloud != nullptr) {
    pcl::fromROSMsg(*velodynePointCloud, *pc);
    curMsg++;
  }
  else return nullptr;

  double et = getTsNow();
  debug_print("Each scan read time :%f", et-st);

  return pc;
}


PclPcXYZI HDL64PCReader::getNextPCI()
{
  double st = getTsNow();
  sensor_msgs::PointCloud2::ConstPtr velodynePointCloud = curMsg->instantiate<sensor_msgs::PointCloud2>();

  PclPcXYZI pc(new pcl::PointCloud<PclXYZI>);

  if(velodynePointCloud != nullptr) {
    pcl::fromROSMsg(*velodynePointCloud, *pc);
    curMsg++;
  }
  else return nullptr;

  double et = getTsNow();
  debug_print("Each scan read time :%f", et-st);

  return pc;
}


void HDL64PCReader::printPCInfo(PclPcXYZ pc)
{
  debug_print("PC Info: elem(%ld) size(%ld)", pc->size(), pc->size()*sizeof(PclXYZ));
}


void HDL64PCReader::printPCInfo(PclPcXYZI pc)
{
  debug_print("PC Info: elem(%ld) size(%ld)", pc->size(), pc->size()*sizeof(PclXYZI));
}

