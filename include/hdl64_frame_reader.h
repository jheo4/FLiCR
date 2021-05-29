#include <bits/stdc++.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#ifndef __PCC_BAGREADER__
#define __PCC_BAGREADER__

class BagReader
{
  protected:
    rosbag::Bag            bag;
    rosbag::View           *view;
    rosbag::View::iterator curMsg;

  public:
    BagReader();
    ~BagReader();
    BagReader(std::string bagFile, std::string topic);
    bool openBag(std::string bagFile, std::string topic);
    void clearSession();
};

#endif

