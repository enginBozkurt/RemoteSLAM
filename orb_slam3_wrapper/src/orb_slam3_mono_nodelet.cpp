#include "orb_slam3_wrapper/orb_slam3_mono_nodelet.h"

namespace orb_slam3_wrapper {
OrbSlam3MonoNodelet::OrbSlam3MonoNodelet() : is_running_(false) {}

OrbSlam3MonoNodelet::~OrbSlam3MonoNodelet() 
{
  if (is_running_) 
  {
    NODELET_WARN("Stopping ORB-SLAM3 mono wrapper nodelet.");
    delete orb_slam3_mono_;
    is_running_ = false;
  }
}

void OrbSlam3MonoNodelet::onInit() 
{
  NODELET_INFO("OnInit - initializing nodelet");
  nh_ = getNodeHandle();
  orb_slam3_mono_ = new orb_slam3_wrapper::OrbSlam3Mono(nh_);
  is_running_ = true;
}
} // namespace orb_slam3_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orb_slam3_wrapper::OrbSlam3MonoNodelet, nodelet::Nodelet)
