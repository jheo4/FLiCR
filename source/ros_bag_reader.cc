#include "ros_bag_reader.h"

namespace pcc {
  ROSBagReader::ROSBagReader()
  {
    view = nullptr;
  }


  ROSBagReader::ROSBagReader(std::string bagFile, std::vector<std::string> topics)
  {
    openBag(bagFile);
  }


  ROSBagReader::~ROSBagReader()
  {
    closeBag();
  }


  bool ROSBagReader::openBag(std::string bagFile)
  {
    closeBag();

    bag.open(bagFile, rosbag::bagmode::Read);
    if(bag.isOpen() == false) {
      std::cerr << "Bag file is failed to open." << std::endl;
      return false;
    }
    return true;
  }


  void ROSBagReader::addTopic(std::string topic)
  {
    bagTopics.push_back(topic);
  }


  bool ROSBagReader::startView()
  {
    if(!bag.isOpen()) {
      std::cerr << "Bag file is not open" << std::endl;
      return false;
    }

    if(bagTopics.size() > 0) {
      view = new rosbag::View(bag, rosbag::TopicQuery(bagTopics));
      return true;
    }
    else {
      std::cerr << "Add topics before starting view" << std::endl;
      return false;
    }
  }


  void ROSBagReader::stopView()
  {
    if(view) {
      std::cerr << "Clear rosbag::View" << std::endl;
      delete view; view = nullptr;
    }
  }


  const rosbag::View* ROSBagReader::getView() {
    if(view) {
      return view;
    }
    else if(startView()) {
      return view;
    }
    else return nullptr;
  }


  void ROSBagReader::closeBag()
  {
    stopView();
    if(bag.isOpen()) {
      std::cerr << "Clear rosbag::Bag" << std::endl;
      bag.close();
    }
  }
}

