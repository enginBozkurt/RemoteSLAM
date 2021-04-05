#ifndef ORB_SLAM3_WRAPPER_H
#define ORB_SLAM3_WRAPPER_H

#include <include/Converter.h>
#include <include/System.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orb_slam3_wrapper
{
class OrbSlam3Mono
{
public:
  OrbSlam3Mono(ros::NodeHandle nh);
  ~OrbSlam3Mono();

private:
  void GrabMono(const sensor_msgs::ImageConstPtr& img);
  void publishPose(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr);
  void initializeSubscribers();
  void initializePublishers();
  void internalInit();
  void readNodeParams();

  ros::NodeHandle nh_;
  ORB_SLAM3::System* mono_slam_;
  ros::Subscriber image_sub_;
  ros::Publisher pose_pub_;
  ros::Time prev_stamp;

  // Parameters
  std::string camera_frame_;
  std::string camera_optical_frame_;
  std::string pose_topic_;
  std::string camera_topic_;
  std::string path_vocabulary_;
  std::string path_settings_;
  bool use_viewer_;
  bool publish_in_optical_frame_;
  double var_x_;
  double var_y_;
  double var_z_;
  double var_roll_;
  double var_pitch_;
  double var_yaw_;
  enum PoseIdx
  {
    X,
    Y,
    Z,
    ROLL,
    PITCH,
    YAW
  };
  const int VAR_IDX_OFFSET = 6;
  double variance_[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
};
}  // namespace orb_slam3_wrapper

#endif  //ORB_SLAM3_WRAPPER_H
