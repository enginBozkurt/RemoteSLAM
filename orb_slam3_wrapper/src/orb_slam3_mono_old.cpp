#include "orb_slam3_wrapper/orb_slam3_mono_old.h"

namespace orb_slam3_wrapper {
OrbSlam3Mono::OrbSlam3Mono(ros::NodeHandle nh)
    : nh_(nh),
      path_vocabulary_("/home/user/lab3/3rdparty/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
      path_settings_("config/kitty.yaml"),
      use_viewer_(false),
      publish_in_optical_frame_(false),
      var_x_(0.0),
      var_y_(0.0),
      var_z_(0.0),
      var_roll_(0.0),
      var_pitch_(0.0),
      var_yaw_(0.0) {

  readNodeParams();
  internalInit();
  initializeSubscribers();
  initializePublishers();

  ros::spin();
}

OrbSlam3Mono::~OrbSlam3Mono() 
{
  mono_slam_->Shutdown();
}

void OrbSlam3Mono::readNodeParams() 
{
  nh_.param<std::string>("camera_frame", camera_frame_, camera_frame_);
  ROS_INFO_STREAM("camera_frame: " << camera_frame_);
  nh_.param<std::string>("camera_optical_frame", camera_optical_frame_, camera_optical_frame_);
  ROS_INFO_STREAM("optical frame: " << camera_optical_frame_);
  nh_.param<std::string>("camera_topic", camera_topic_, camera_topic_);
  ROS_INFO_STREAM("camera topic: " << camera_topic_);
  nh_.param<std::string>("pose_topic", pose_topic_, "orb_slam3_pose");// name in launch file, name of variable, default value
  ROS_INFO_STREAM("pose topic: " << pose_topic_);
  nh_.param<std::string>("path_vocabulary", path_vocabulary_, path_vocabulary_);
  ROS_INFO_STREAM("path vocabulary: " << path_vocabulary_);
  nh_.param<std::string>("path_settings", path_settings_, path_settings_);
  ROS_INFO_STREAM("path settings: " << path_settings_);
  nh_.param<bool>("use_viewer", use_viewer_, use_viewer_);

  nh_.param<double>("orb_var_x", var_x_, var_x_);
  nh_.param<double>("orb_var_y", var_y_, var_y_);
  nh_.param<double>("orb_var_z", var_z_, var_z_);
  nh_.param<double>("orb_var_roll", var_roll_, var_roll_);
  nh_.param<double>("orb_var_pitch", var_pitch_, var_pitch_);
  nh_.param<double>("orb_var_yaw", var_yaw_, var_yaw_);
}

void OrbSlam3Mono::internalInit() 
{
  variance_[X] = var_x_;
  variance_[Y] = var_y_;
  variance_[Z] = var_z_;
  variance_[ROLL] = var_roll_;
  variance_[PITCH] = var_pitch_;
  variance_[YAW] = var_yaw_;

  mono_slam_ =
      new ORB_SLAM3::System(path_vocabulary_, path_settings_, ORB_SLAM3::System::MONOCULAR, use_viewer_);
}

void OrbSlam3Mono::initializeSubscribers()
 {
  image_sub_ = nh_.subscribe(camera_topic_, 1, &OrbSlam3Mono::GrabMono, this);

}

void OrbSlam3Mono::initializePublishers() {
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 1);
}

void OrbSlam3Mono::GrabMono(const sensor_msgs::ImageConstPtr &msg) 
{
  ROS_INFO_STREAM_ONCE("Start grabbing images...");
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat Tcw = mono_slam_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec()).clone();
  // If there is no solution
  if (Tcw.empty()) {
    ROS_WARN_THROTTLE(1, "no solution");
    return;
  }
  publishPose(Tcw, cv_ptr);

  {
    ros::Time stamp = ros::Time::now();
    float temp = stamp.toSec() - prev_stamp.toSec();
    prev_stamp = ros::Time::now();
    std::cout << "delta time between frames  " << temp << std::endl;
  }

}

void OrbSlam3Mono::publishPose(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr) 
{
  cv::Mat rotation(3, 3, CV_32F);
  cv::Mat translation(3, 1, CV_32F);

  rotation = Tcw.rowRange(0, 3).colRange(0, 3).t();
  translation = rotation * Tcw.rowRange(0, 3).col(3);

  tf::Matrix3x3 tf_camera_rotation(rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
                                   rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
                                   rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2)
  );

  tf::Vector3 tf_camera_translation(translation.at<float>(0), translation.at<float>(1), translation.at<float>(2)); 

  const tf::Matrix3x3 Rx(1, 0, 0,
                         0, 0, -1,
                         0, 1, 0);

  const tf::Matrix3x3 Rz(0, -1, 0,
                         1, 0, 0,
                         0, 0, 1);

  const tf::Matrix3x3 invX(-1, 0, 0,
                           0, 1, 0,
                           0, 0, 1);

  const tf::Matrix3x3 invYZ(1, 0, 0,
                            0, -1, 0,
                            0, 0, -1);

  tf_camera_rotation = Rx * tf_camera_rotation;
  tf_camera_rotation = Rz * tf_camera_rotation;
  tf_camera_translation = Rx * tf_camera_translation;
  tf_camera_translation = Rz * tf_camera_translation;

  tf_camera_rotation = invYZ * tf_camera_rotation;
  tf_camera_translation = invX * tf_camera_translation;

  tf::Transform transform(tf_camera_rotation, tf_camera_translation);

  tf::Quaternion q;
  tf_camera_rotation.getRotation(q);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, cv_ptr->header.stamp, "world", camera_optical_frame_)); // from "world" to camera_optical_frame_

  //ROS_DEBUG("temp1");
  geometry_msgs::PoseWithCovarianceStamped current_pose_;
  current_pose_.header.stamp = cv_ptr->header.stamp;
  current_pose_.header.frame_id = camera_frame_;
  current_pose_.pose.pose.orientation.x = q[0];
  current_pose_.pose.pose.orientation.y = q[1];
  current_pose_.pose.pose.orientation.z = q[2];
  current_pose_.pose.pose.orientation.w = q[3];
  current_pose_.pose.pose.position.x = tf_camera_translation.getX();
  current_pose_.pose.pose.position.y = tf_camera_translation.getY();
  current_pose_.pose.pose.position.z = tf_camera_translation.getZ();
  pose_pub_.publish(current_pose_);
  //ROS_DEBUG("temp2");
}
}  // namespace orb_slam3_wrapper
