#include <bits/stdc++.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <bag_reader.h>
#include <types.h>
#include <defs.h>
#include <utils.h>

#ifndef __PCC_OXTSREADER__
#define __PCC_OXTSREADER__

class OxtsReader: public BagReader
{
  protected:
    rosbag::View           *gpsView;
    rosbag::View::iterator curGpsMsg;
    int                    numGpsMsg;

  public:
    OxtsReader();
    OxtsReader(std::string bagFile, std::string imuTopic, std::string gpsTopic);
    OxtsMsg getNextMsg();
};

#endif

