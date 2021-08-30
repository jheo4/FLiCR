#include <oxts_reader.h>


OxtsReader::OxtsReader(): BagReader()
{
  view = nullptr;
  gpsView = nullptr;
  numGpsMsg = 0;
}


OxtsReader::OxtsReader(std::string bagFile, std::string imuTopic, std::string gpsTopic): BagReader(bagFile, imuTopic)
{
  openBag(bagFile, imuTopic);

  gpsView = new rosbag::View(bag, rosbag::TopicQuery(gpsTopic));
  curGpsMsg = gpsView->begin();
  numGpsMsg = gpsView->size();
}


OxtsMsg OxtsReader::getNextMsg()
{
  //double st = getTsNow();
  sensor_msgs::ImuConstPtr oxtsImu = curMsg->instantiate<sensor_msgs::Imu>();
  geometry_msgs::TwistStampedConstPtr oxtsGps = curGpsMsg->instantiate<geometry_msgs::TwistStamped>();

  OxtsMsg oxtsMsg;
  if(oxtsImu != nullptr && oxtsGps != nullptr)
  {
    oxtsMsg.orientation.x = oxtsImu->orientation.x;
    oxtsMsg.orientation.y = oxtsImu->orientation.y;
    oxtsMsg.orientation.z = oxtsImu->orientation.z;
    oxtsMsg.orientation.w = oxtsImu->orientation.w;

    oxtsMsg.angularVelocity.x = oxtsImu->angular_velocity.x;
    oxtsMsg.angularVelocity.y = oxtsImu->angular_velocity.y;
    oxtsMsg.angularVelocity.z = oxtsImu->angular_velocity.z;

    oxtsMsg.linearAcceleration.x = oxtsImu->linear_acceleration.x;
    oxtsMsg.linearAcceleration.y = oxtsImu->linear_acceleration.y;
    oxtsMsg.linearAcceleration.z = oxtsImu->linear_acceleration.z;

    oxtsMsg.gpsVelocity.x = oxtsGps->twist.linear.x;
    oxtsMsg.gpsVelocity.y = oxtsGps->twist.linear.y;
    oxtsMsg.gpsVelocity.z = oxtsGps->twist.linear.z;

    curMsg++;
    curGpsMsg++;
  }

  return oxtsMsg;
}

