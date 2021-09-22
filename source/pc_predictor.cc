#include <pc_predictor.h>
#include <utils.h>
#include <defs.h>


PclPcXYZ PcPredictor::predictNextPc(PclPcXYZ origPc, VecXyz velocity)
{
  PclPcXYZ predPc(new pcl::PointCloud<PclXYZ>);

  for(auto oP: origPc->points)
  {
    pcl::PointXYZ pP;
    pP.x = oP.x + (velocity.x/KITTI_DATASET_FREQUENCY);
    pP.y = oP.y + (velocity.y/KITTI_DATASET_FREQUENCY);
    pP.z = oP.z + (velocity.z/KITTI_DATASET_FREQUENCY);
    //pP.z = oP.z;

    predPc->push_back(pP);
  }

  return predPc;
}

