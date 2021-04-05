#include "orb_slam3_wrapper/orb_slam3_mono_old.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "orb_slam3_mono_node");
  ros::NodeHandle nh;

  orb_slam3_wrapper::OrbSlam3Mono orb_slam3_mono(nh);
  while (ros::ok())
  { /* do nothing */
  }
}