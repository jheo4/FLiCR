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

std::vector<HDL64PointCloud>* HDL64PCReader::getNextPC()
{
  double st = getTsNow();
  sensor_msgs::PointCloud2::ConstPtr velodynePointCloud = curMsg->instantiate<sensor_msgs::PointCloud2>();

  std::vector<HDL64PointCloud> *pc = new std::vector<HDL64PointCloud>;
  if(velodynePointCloud != nullptr) {
    HDL64PointCloud *elem = (HDL64PointCloud*)velodynePointCloud->data.data();

    for(uint32_t i = 0; i < velodynePointCloud->width; i++) {
      pc->push_back(*elem);
      elem++;
    }
    curMsg++;
  }
  else return nullptr;

  double et = getTsNow();
  debug_print("Each scan read time :%f", et-st);

  return pc;
}

void HDL64PCReader::printPCInfo(std::vector<HDL64PointCloud> &pc)
{
  debug_print("PC Info: elem(%ld) size(%ld)", pc.size(), pc.size()*sizeof(HDL64PointCloud));
}

