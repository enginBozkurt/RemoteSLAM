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



using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;

    ros::Time  prev_stamp;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam3_mono");
    
    ros::NodeHandle nodeHandler; // ("~");  //Yoni
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
    nodeHandler.param<std::string>("path_vocabulary", path_vocabulary_, path_vocabulary_);
    ROS_INFO_STREAM("path vocabulary: " << path_vocabulary_);
    nodeHandler.param<std::string>("path_settings", path_settings_, path_settings_);
    ROS_INFO_STREAM("path settings: " << path_settings_);
    //nh.param<bool>("use_viewer", use_viewer_, use_viewer_);.


    ORB_SLAM3::System SLAM(path_vocabulary_, path_settings_,ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    
    image_transport::ImageTransport it(nodeHandler);
    image_transport::Subscriber  image_sub_ = it.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    //ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    {
        ros::Time stamp = ros::Time::now();
        float temp = stamp.toSec() - prev_stamp.toSec();
        prev_stamp = ros::Time::now();
        std::cout << "delta time between frames  " << temp << std::endl;
     }
}


