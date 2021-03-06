#ifndef BLUEFOX3_H
#define BLUEFOX3_H


// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Header.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

// Msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Bluefox stuff
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_helper.h>
#include <mvDeviceManager/Include/mvDeviceManager.h>

// std
#include <mutex>
#include <functional>

// local stuff
#include <bluefox3/Bluefox3Config.h>



namespace bluefox3
{
  

  float prev_time;
  struct ThreadParameter
  {
    Device* cameraDevice_ptr;
    unsigned int requestsCaptured;
    Statistics statistics;
    explicit ThreadParameter(Device* cameraDevice_ptr) : cameraDevice_ptr(cameraDevice_ptr), requestsCaptured(0), statistics(cameraDevice_ptr)
    {
    }
    ThreadParameter(const ThreadParameter& src) = delete;
    ThreadParameter& operator=(const ThreadParameter& rhs) = delete;
  };

  

  /* class Bluefox3 //{ */

  class Bluefox3 : public nodelet::Nodelet
  {
  public:
    Bluefox3() : m_node_name("Bluefox3"), m_running(false) {};
    ~Bluefox3();
    virtual void onInit();
    void printDevices();

  private:
    const std::string m_node_name;
    bool m_running;
    ros::Time prev_time;

    ros::Subscriber m_sub;
    void triggerCallback(const std_msgs::HeaderConstPtr msgPtr);
    std::queue<std_msgs::Header> m_trigger_queue;

  private:
    DeviceManager m_devMgr;
    Device* m_cameraDevice;
    std::shared_ptr<GenICam::ImageFormatControl> m_GenICamImageFormat_ptr;
    std::shared_ptr<ImageDestination> m_destinationFormat_ptr;
    std::shared_ptr<GenICam::AcquisitionControl> m_GenICamACQ_ptr;
    std::shared_ptr<GenICam::CounterAndTimerControl> m_GenICamCounterTimer_ptr;
    std::shared_ptr<ImageProcessing> m_imgProc_ptr;
    std::shared_ptr<GenICam::ChunkDataControl> m_GenICamImageChunk_ptr;
    std::shared_ptr<ThreadParameter> m_threadParam_ptr;
    std::shared_ptr<helper::RequestProvider> requestProvider_ptr;
    std::shared_ptr<GenICam::AnalogControl> m_GenICamAnalog_ptr;

  private:
    std::mutex m_pub_mtx;
    image_transport::CameraPublisher m_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfoMgr_ptr;

  private:
    std::string pixelFormatToEncoding(const PropertyIImageBufferPixelFormat& pixel_format);
    std::string bayerPatternToEncoding(const PropertyIBayerMosaicParity& bayer_pattern, int bytes_per_pixel);

  private:
    // | --------------------- Helper methods --------------------- |
    void imageCallback(std::shared_ptr<Request> pRequest, std::shared_ptr<ThreadParameter> threadParameter_ptr);
    void dynRecCallback(bluefox3::Bluefox3Config& cfg, [[maybe_unused]] uint32_t level);

    template <typename T>
    bool getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out);

    template <typename T>
    bool getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out, const T& param_default);

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */

    std::string m_frame_id;

    boost::recursive_mutex m_dynRecServer_mtx;
    std::shared_ptr<dynamic_reconfigure::Server<Bluefox3Config>> m_dynRecServer_ptr;
    Bluefox3Config m_lastCfg;
  };

  

}  // namespace bluefox3

#endif  // #ifndef BALLOONFILTER
