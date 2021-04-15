#include <bits/stdc++.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace pcc {
  class ROSBagReader {
    protected:
      rosbag::Bag bag;
      rosbag::View *view;
      std::vector<std::string> bagTopics;


    public:
      ROSBagReader();
      ROSBagReader(std::string bagFile, std::vector<std::string> topics);
      bool openBag(std::string bagFile);
      void addTopic(std::string topic);
      bool startView();
      const rosbag::View* getView();
      void stopView();
      void closeBag();
      ~ROSBagReader();
  };
}

