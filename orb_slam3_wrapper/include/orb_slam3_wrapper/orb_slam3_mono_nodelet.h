#ifndef PROJECT_ORB_SLAM3_MONO_NODELET_H
#define PROJECT_ORB_SLAM3_MONO_NODELET_H

#include "orb_slam3_mono_old.h"
#include <nodelet/nodelet.h>

namespace orb_slam3_wrapper {
class OrbSlam3MonoNodelet : public nodelet::Nodelet 
{
public:
  OrbSlam3MonoNodelet();
  ~OrbSlam3MonoNodelet();
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool is_running_;
  orb_slam3_wrapper::OrbSlam3Mono *orb_slam3_mono_;
};
} // namespace orb_slam3_wrapper

#endif // PROJECT_ORB_SLAM3_MONO_NODELET_H
