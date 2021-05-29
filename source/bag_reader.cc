#include <bag_reader.h>

BagReader::BagReader()
{
  view = nullptr;
}

BagReader::BagReader(std::string bagFile, std::string topic)
{
  view = nullptr;
  openBag(bagFile, topic);
}

BagReader::~BagReader()
{
  clearSession();
}

bool BagReader::openBag(std::string bagFile, std::string topic)
{
  clearSession();

  bag.open(bagFile, rosbag::bagmode::Read);
  if(bag.isOpen() == false) {
    return false;
  }

  view = new rosbag::View(bag, rosbag::TopicQuery(topic));
  curMsg = view->begin();

  return true;
}

void BagReader::clearSession()
{
  if(view) {
    delete view; view = nullptr;
  }
  if(bag.isOpen()) {
    bag.close();
  }
}

