//#include "orb_slam3_wrapper/orb_slam3_mono_inertial.h"
#include <include/Converter.h>
#include <include/System.h>
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



#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<sensor_msgs/Imu.h>
//#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;


class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe, ros::NodeHandle &node_handle): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe), mnode_handle(node_handle){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    ros::NodeHandle mnode_handle;

     
    //pose_pub_ = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/pose_topic", 1);
    ros::Publisher pose_pub_ ;// = node_handle_.advertise<geometry_msgs::PoseStamped> ("/pose", 1); 
    //pose_pub_  = node_handle_.advertise<geometry_msgs::PoseStamped> ("/pose", 1);

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orb_slam3_mono_inertial");
  ros::NodeHandle nh; // ("~");  //Yoni
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  
  std::string path_vocabulary_, path_settings_, use_viewer_;
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  /*nh_.param<std::string>("camera_frame", camera_frame_, camera_frame_);
  ROS_INFO_STREAM("camera_frame: " << camera_frame_);
  nh_.param<std::string>("camera_optical_frame", camera_optical_frame_, camera_optical_frame_);
  ROS_INFO_STREAM("optical frame: " << camera_optical_frame_);
  nh_.param<std::string>("camera_topic", camera_topic_, camera_topic_);
  ROS_INFO_STREAM("camera topic: " << camera_topic_);
  nh_.param<std::string>("pose_topic", pose_topic_, "orb_slam3_pose");// name in launch file, name of variable, default value
  ROS_INFO_STREAM("pose topic: " << pose_topic_);*/
  nh.param<std::string>("path_vocabulary", path_vocabulary_, path_vocabulary_);
  ROS_INFO_STREAM("path vocabulary: " << path_vocabulary_);
  nh.param<std::string>("path_settings", path_settings_, path_settings_);
  ROS_INFO_STREAM("path settings: " << path_settings_);
  //nh.param<bool>("use_viewer", use_viewer_, use_viewer_);

  ORB_SLAM3::System SLAM(path_vocabulary_, path_settings_, ORB_SLAM3::System::IMU_MONOCULAR,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual,nh); // TODO
  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img0 = nh.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);

  
  //pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
   pose_pub_ = mnode_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_topic", 1);
   ros::Time prev_stamp;
   //queue<sensor_msgs::ImageConstPtr> img0Buf;
    //std::mutex mBufMutex;

  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    int num_of_imu_meas = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec()- 0.015; //Yoni
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
        this->mBufMutex.lock();
        im = GetImage(img0Buf.front());
        img0Buf.pop();
        this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
          num_of_imu_meas ++;
        }
        std::cout << "num_of_imu_meas  " << num_of_imu_meas << std::endl;
        num_of_imu_meas = 0;
      }
      else
      {
          std::cout << "This can not happennnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn" << std::endl;
      }
      
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

      {
          ros::Time stamp = ros::Time::now();
          float temp = stamp.toSec() - prev_stamp.toSec();
          prev_stamp = ros::Time::now();
          std::cout << "delta time between frames  " << temp << std::endl;
      }
      cv::Mat Tcw = mpSLAM->TrackMonocular(im,tIm,vImuMeas);

      if (Tcw.empty()) 
       {
           ROS_WARN_THROTTLE(1, "no solution");
           //return;
       }
      else
      {
            cv::Mat rotation(3,3,CV_32F);
            cv::Mat translation(3,1,CV_32F);

            rotation = Tcw.rowRange(0,3).colRange(0,3);
            translation = Tcw.rowRange(0,3).col(3);

            tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                                rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                                rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                            );

            tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

            //Coordinate transformation matrix from orb coordinate system to ros coordinate system
            const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                                -1, 0, 0,
                                                0,-1, 0);

            //Transform from orb coordinate system to ros coordinate system on camera coordinates
            tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
            tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

            //Inverse matrix
            tf_camera_rotation = tf_camera_rotation.transpose();
            tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

            //Transform from orb coordinate system to ros coordinate system on map coordinates
            tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
            tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

            tf::Transform transform(tf_camera_rotation, tf_camera_translation);

            //tf::Quaternion q;
            //tf_camera_rotation.getRotation(q);

            //static tf::TransformBroadcaster br;
            //br.sendTransform(tf::StampedTransform(transform, cv_ptr->header.stamp, "world", camera_optical_frame_)); // from "world" to camera_optical_frame_


            geometry_msgs::PoseWithCovarianceStamped current_pose_;
            //current_pose_.header.stamp = cv_ptr->header.stamp;
            //current_pose_.header.frame_id = camera_frame_;
            //current_pose_.pose.pose.orientation.x = q[0];
            //current_pose_.pose.pose.orientation.y = q[1];
            //current_pose_.pose.pose.orientation.z = q[2];
            //current_pose_.pose.pose.orientation.w = q[3];
            current_pose_.pose.pose.position.x = tf_camera_translation.getX();
            current_pose_.pose.pose.position.y = tf_camera_translation.getY();
            current_pose_.pose.pose.position.z = tf_camera_translation.getZ();
            pose_pub_.publish(current_pose_);
      }
           

    }    


    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

/////////////




/*void OrbSlam3Mono::GrabMono(const sensor_msgs::ImageConstPtr &msg) 
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
}*/

/*void OrbSlam3Mono::publishPose(cv::Mat Tcw) //, cv_bridge::CvImageConstPtr cv_ptr) 
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

  //tf::Transform transform(tf_camera_rotation, tf_camera_translation);

  //tf::Quaternion q;
  //tf_camera_rotation.getRotation(q);

  //static tf::TransformBroadcaster br;
  //br.sendTransform(tf::StampedTransform(transform, cv_ptr->header.stamp, "world", camera_optical_frame_)); // from "world" to camera_optical_frame_

  //ROS_DEBUG("temp1");
  geometry_msgs::PoseWithCovarianceStamped current_pose_;
  //current_pose_.header.stamp = cv_ptr->header.stamp;
  //current_pose_.header.frame_id = camera_frame_;
  //current_pose_.pose.pose.orientation.x = q[0];
  //current_pose_.pose.pose.orientation.y = q[1];
  //current_pose_.pose.pose.orientation.z = q[2];
  //current_pose_.pose.pose.orientation.w = q[3];
  current_pose_.pose.pose.position.x = tf_camera_translation.getX();
  current_pose_.pose.pose.position.y = tf_camera_translation.getY();
  current_pose_.pose.pose.position.z = tf_camera_translation.getZ();
  pose_pub_.publish(current_pose_);
  //ROS_DEBUG("temp2");
}*/


//}  // namespace orb_slam3_wrapper
