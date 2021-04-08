#include <bluefox3/bluefox3.h>

namespace bluefox3
{
	ros::Time groundTime;
	int first_frame_seq = -1;

        /* triggerCallback() method //{ */
	void Bluefox3::triggerCallback(const std_msgs::HeaderConstPtr msgPtr)
	{

	//if (strcmp(m_GenICamACQ_ptr -> triggerMode.readS().c_str(), "On") == 0) {
		// Make a copy for Queue
		std_msgs::Header tmp_header;
		tmp_header.seq = msgPtr->seq;
		tmp_header.stamp = msgPtr->stamp;
		tmp_header.frame_id = msgPtr->frame_id;
		
		if (first_frame_seq == -1) 
		{
			groundTime = tmp_header.stamp;
			first_frame_seq = tmp_header.seq;
			std::lock_guard<std::mutex> lck(m_pub_mtx);
			m_trigger_queue.push(tmp_header);
			ROS_INFO("first_frame_seq = -1");
		}
		
		
	  m_GenICamACQ_ptr -> triggerSoftware.call();
		int tcount = msgPtr->seq;
		std::cout << "Trigger number: " << tcount << std::endl;
		
		//ROS_INFO("trigger temp");
		
	  ROS_INFO("trigger sequence number: [%d]", msgPtr->seq);
		
	}
	
	
    /* pixelFormatToEncoding() function //{ */

    std::string Bluefox3::pixelFormatToEncoding(const PropertyIImageBufferPixelFormat& pixel_format)
    {
      std::string ret = "unknown";
      switch (pixel_format.read())
      {
        case ibpfMono8:
        //ROS_INFO("mono8 for sure ");
        ret = sensor_msgs::image_encodings::MONO8;
        break;
        case ibpfMono10:
        case ibpfMono12:
        case ibpfMono14:
        case ibpfMono16:
        ret = sensor_msgs::image_encodings::MONO16;
        break;
        case ibpfMono32:
        ret = sensor_msgs::image_encodings::TYPE_32FC1;
        break;
        case ibpfBGR888Packed:
        ret = sensor_msgs::image_encodings::BGR8;
        case ibpfRGB888Packed:
        ret = sensor_msgs::image_encodings::RGB8;
        break;
        case ibpfRGBx888Packed:
        ret = sensor_msgs::image_encodings::RGBA8;
        break;
        case ibpfRGB101010Packed:
        case ibpfRGB121212Packed:
        case ibpfRGB141414Packed:
        case ibpfRGB161616Packed:
        ret = sensor_msgs::image_encodings::RGB16;
        break;
        default:
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown pixel format: '" << pixel_format << "'");
        break;
      }
		  return ret;
    }



    /* bayerPatternToEncoding() function //{ */

    std::string Bluefox3::bayerPatternToEncoding(const PropertyIBayerMosaicParity& bayer_pattern, int bytes_per_pixel)
    {
      if (bytes_per_pixel == 1)
      {
        switch (bayer_pattern)
        {
          case bmpRG:
            return sensor_msgs::image_encodings::BAYER_RGGB8;
          case bmpGB:
            return sensor_msgs::image_encodings::BAYER_GBRG8;
          case bmpGR:
            return sensor_msgs::image_encodings::BAYER_GRBG8;
          case bmpBG:
            return sensor_msgs::image_encodings::BAYER_BGGR8;
          default:
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown bayer pattern: '" << bayer_pattern << "'");
        }
      } 
      else if (bytes_per_pixel == 2)
      {
        switch (bayer_pattern)
        {
          case bmpRG:
            return sensor_msgs::image_encodings::BAYER_RGGB16;
          case bmpGB:
            return sensor_msgs::image_encodings::BAYER_GBRG16;
          case bmpGR:
            return sensor_msgs::image_encodings::BAYER_GRBG16;
          case bmpBG:
            return sensor_msgs::image_encodings::BAYER_BGGR16;
          default:
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: Unknown bayer pattern: '" << bayer_pattern << "'");
        }
      }
      return "unknownBayer";
    }

    
    
    /* imageCallback() method //{ */

    void Bluefox3::imageCallback(std::shared_ptr<Request> request_ptr, std::shared_ptr<ThreadParameter> threadParameter_ptr)
    {
      
      const ros::Time cbk_time = ros::Time::now();
      
      //ROS_INFO_STREAM("imageCallback");
      threadParameter_ptr->requestsCaptured++;
      // display some statistical information every 100th image
      //const Statistics& s = threadParameter_ptr->statistics;
      //const auto capture_time_corrected(s.captureTime_s.readS());///10.0); // why division ??? captureTime_s.readS()
      //const auto exposure_time_corrected(request_ptr->chunkExposureTime.readS());///1000000/2);//chunkExposureTime.read
      
      //if (threadParameter_ptr->requestsCaptured % 1 == 0)
      //{
        /*ROS_INFO_STREAM_THROTTLE(2.0, "[" << m_node_name.c_str() << "]: "
                        << "Info from " << threadParameter_ptr->cameraDevice_ptr->serial.read() << ": " << s.framesPerSecond.name() << ": "
                        << s.framesPerSecond.readS() << ", " << s.errorCount.name() << ": " << s.errorCount.readS() << ", "
                        << s.captureTime_s.name() << ": " << capture_time_corrected << ", Exposure: " << request_ptr->chunkExposureTime.readS());
      */
          
      //}
      
      if (request_ptr->isOK())
      {
          const auto imw = request_ptr->imageWidth.read();
          const auto imh = request_ptr->imageHeight.read();
          const auto imdata = request_ptr->imageData.read();
          const auto imstep = request_ptr->imageLinePitch.read();

          /*if (threadParameter_ptr->requestsCaptured % 50 == 0)
          {
             ROS_INFO("imageWidth %4d", imw);
             ROS_INFO("imageHeight %4d", imh);
             ROS_INFO("image line pitch %4d", imstep);
          }
          */
          std::string encoding;
          const auto bayer_mosaic_parity = request_ptr->imageBayerMosaicParity;
          if (bayer_mosaic_parity.read() != bmpUndefined)
          {
            // Bayer pattern
            const auto bytes_per_pixel = request_ptr->imageBytesPerPixel.read();
            encoding = bayerPatternToEncoding(bayer_mosaic_parity, bytes_per_pixel);
            ROS_INFO_STREAM("wrong enoding...........");
          } 
          else
          {
            encoding = pixelFormatToEncoding(request_ptr->imagePixelFormat);
            //ROS_INFO_STREAM("encoding = " << encoding);
            //ROS_INFO("image line pitch %4d", imstep);
            //ROS_INFO_STREAM("encoding", encoding);
          }
          
          sensor_msgs::Image image_msg;
          sensor_msgs::fillImage(image_msg, encoding, imh, imw, imstep, imdata);
          ros::Time stamp =  cbk_time;// - ros::Duration(10); //Yoni - capture_time_corrected + exposure_time_corrected;
          image_msg.header.stamp = stamp;
          image_msg.header.frame_id = m_frame_id;

          sensor_msgs::CameraInfo cinfo_msg = m_cinfoMgr_ptr->getCameraInfo();
         
          //if (!m_trigger_queue.empty()) 
          //{
            //int fcount = s.frameCount.read();
            //ros::Duration trigger_offset = ros::Duration((fcount - first_frame_seq - 1) * 0.05);
            
            //ROS_INFO("[%d]", m_trigger_queue.size());
            //ROS_INFO("666_b");
            //image_msg.header.stamp = m_trigger_queue.back().stamp + exposure_time_corrected;
            //image_msg.header.stamp = groundTime + trigger_offset ; //Yoni exposure_time_corrected;
          //}
          //else 
            //std::cout << "queue empty!" << std::endl;
        
          //std::lock_guard<std::mutex> lck(m_pub_mtx);  //std::mutex m_pub_mtx;
          cinfo_msg.header = image_msg.header;
          m_pub.publish(image_msg, cinfo_msg);
       
          //ROS_INFO("c_sec1 %f", cbk_time.toSec());
          //ROS_INFO("c_sec2 %f", cbk_time.toSec()-prev_time.toSec());
          {
            float temp;
            temp = cbk_time.toSec()-prev_time.toSec();
            if ((temp>0.06)|(temp<0.04))
            {
                //ROS_INFO("c_sec1 %f", cbk_time.toSec());
                ROS_INFO("problem with synchronization %f", temp);
            }
          }
          prev_time = cbk_time;
              
      } 
      else
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name.c_str() << "]: Error capturing image: " << request_ptr->requestResult.readS());
      }
    }

    
    /* printDevices() method //{ */
    void Bluefox3::printDevices()
    {
      if (m_devMgr.deviceCount() == 0)
      {
        ROS_INFO("[%s]: No devices found!", m_node_name.c_str());
        return;
      }
      ROS_INFO("[%s]: Listing all available devices:", m_node_name.c_str());
      // show all devices
      for (unsigned int i = 0; i < m_devMgr.deviceCount(); i++)
        std::cout << "\t#: " << i << "\t" << (m_devMgr[i])->deviceID.name() << ": " << (m_devMgr[i])->deviceID.readS() << "\t"  << (m_devMgr[i])->family.name() << ": " << (m_devMgr[i])->family.readS() << "\t"  << (m_devMgr[i])->product.name() << ": " << (m_devMgr[i])->product.readS() << "\t"  << (m_devMgr[i])->serial.name() << ": " << (m_devMgr[i])->serial.readS() << std::endl;
    }
    

    /* dynRecCallback() method //{ */
    void Bluefox3::dynRecCallback(bluefox3::Bluefox3Config& cfg, [[maybe_unused]] uint32_t level)
    {
	    ROS_INFO("[%s]: Received dynamic reconfigure callback.", m_node_name.c_str());
	    //m_GenICamACQ_ptr->triggerSelector.writeS( cfg.acq_trigger_selector );
	    //m_GenICamACQ_ptr->triggerSource.writeS( cfg.acq_trigger_source );
	    
	    m_GenICamACQ_ptr->triggerMode.writeS(cfg.acq_trigger_mode);
	    ROS_INFO_STREAM("acq_trigger_enable = " << cfg.acq_trigger_mode);
        m_GenICamACQ_ptr->exposureAuto.writeS(std::string(cfg.acq_exposure_AECMode));
        
	    //m_GenICamACQ_ptr->exposureAuto.writeS(cfg.acq_exposure_AECMode);
	     //m_GenICamACQ_ptr->exposureAuto.writeS(std::string("Off"));
	     //ROS_INFO_STREAM("333");
	     //m_GenICamACQ_ptr->exposureTime.writeS(cfg.acq_exposure_time);

	    //m_destinationFormat_ptr -> pixelFormat.writeS(cfg.dest_pixel_format);
		
       /* try
        {
            m_GenICamACQ_ptr->mvExposureAutoUpperLimit.write(cfg.acq_exp_limit_upper);
            m_GenICamACQ_ptr->mvExposureAutoLowerLimit.write(cfg.acq_exp_limit_lower);
        }
        catch (mvIMPACT::acquire::ImpactAcquireException& e)
        {
            ROS_INFO("[%s]: An error occurred (%s). Auto-Exposure has to be on 'Continuous' for that feature.", m_node_name.c_str(), e.getErrorString().c_str());

        }*/
     }



    template <typename T>
    bool Bluefox3::getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out)
    {
        const bool res = nh.getParam(param_name, param_out);
        if (!res)
          ROS_ERROR_STREAM("[" << m_node_name << "]: Could not load compulsory parameter '" << param_name << "'");
        //else
         // ROS_INFO_STREAM("[" << m_node_name << "]: Loaded parameter '" << param_name << "': " << param_out);
        return res;
    }

    template <typename T>
    bool Bluefox3::getParamCheck(const ros::NodeHandle& nh, const std::string& param_name, T& param_out, const T& param_default)
    {
        const bool res = nh.getParam(param_name, param_out);
        if (!res)
        {
            param_out = param_default;
            //ROS_INFO_STREAM("[" << m_node_name << "]: Loaded parameter '" << param_name << "': " << param_out);
            //ROS_INFO_STREAM("param_out = param_default = " << param_default);
        }
        //elsem_GenICamACQ_ptr
        {
          //ROS_INFO_STREAM("[" << m_node_name << "]: Loaded parameter '" << param_name << "': " << param_out);
        }
        return res;
    }
   

    void Bluefox3::onInit()
    {
        ROS_INFO("[%s]: Initializing", m_node_name.c_str());

        // | ------------------ Begin initialization ------------------ |


        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        ros::NodeHandle _nh = nodelet::Nodelet::getNodeHandle();
        ros::Time::waitForValid();
        printDevices();
        m_dynRecServer_ptr = std::make_shared<dynamic_reconfigure::Server<Bluefox3Config>>(m_dynRecServer_mtx, nh);
        

        /* load parameters //{ */
        bool success = true;
        std::string camera_serial;
        std::string camera_name;
        std::string calib_url;

        std::string imgproc_mirror_mode;
        std::string acq_mode;
        std::string acq_exposure_time;
        std::string acq_autoexp_mode;

        std::string acq_trigger_enable;
        std::string acq_trigger_source;
        std::string acq_trigger_select;
        std::string acq_trigger_activation;

        double acq_autoexp_limit_upper;
        double acq_autoexp_limit_lower;

        // Required
        success = success && getParamCheck(nh, "camera_serial", camera_serial);
        success = success && getParamCheck(nh, "camera_name", camera_name);
        success = success && getParamCheck(nh, "calib_url", calib_url);
        success = success && getParamCheck(nh, "frame_id", m_frame_id);

        // Optional                
        
        getParamCheck(nh, "trigger_activation", acq_trigger_activation, std::string("AnyEdge"));  
        getParamCheck(nh, "exposure_time", acq_exposure_time, std::string("1000"));
        getParamCheck(nh, "exposure_auto_upper_limit", acq_autoexp_limit_upper, 30000.0);
        getParamCheck(nh, "exposure_auto_lower_limit", acq_autoexp_limit_lower, 10000.0);



        if (!success)
        {
          ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
          ros::shutdown();
          return;
        }

        // Setup CameraInfo manager
        m_cinfoMgr_ptr = std::make_shared<camera_info_manager::CameraInfoManager>(nh, camera_name, calib_url);
        m_cinfoMgr_ptr->loadCameraInfo(calib_url);

        // Setup initial dynamic reconfigure parameters
        Bluefox3Config cfg;
        
        	        
        // Image & Destination Format
        // keeping these constant to ensure the rpi can handle the input data stream
        cfg.ifc_pixel_format = std::string("Mono8");
        cfg.ifc_height = std::string("512"); //384 //1024
        cfg.ifc_width = std::string("640"); //516 //1280
        cfg.dest_pixel_format = std::string("Mono8");

        //m_dynRecServer_ptr->updateConfig(cfg);

        /* initialize publishers//{ */

        image_transport::ImageTransport it(nh);
// image_transport::CameraPublisher m_pub;
        m_pub = it.advertiseCamera("image_raw", 1);
        

        /* try to find the camera device //{ */
        m_cameraDevice = m_devMgr.getDeviceBySerial(camera_serial);

        if (m_cameraDevice == nullptr)
        {
          ROS_ERROR("[%s]: Camera with serial '%s' was not found, ending the node", m_node_name.c_str(), camera_serial.c_str());
          ros::shutdown();
          return;
        }

        try
        {
          m_cameraDevice->open();
        }
        catch (mvIMPACT::acquire::ImpactAcquireException& e)
        {
          ROS_ERROR("[%s]: An error occurred while opening the device (%s), ending the node", m_node_name.c_str(), e.getErrorString().c_str());
          ros::shutdown();
          return;
        }


        // Exposure        
        cfg.acq_exposure_time = acq_exposure_time;        
        cfg.acq_exp_limit_upper = acq_autoexp_limit_upper;
        cfg.acq_exp_limit_lower = acq_autoexp_limit_lower;
        // start at Continuous acquisition mode, set up pointer
        m_GenICamACQ_ptr = std::make_shared<GenICam::AcquisitionControl>(m_cameraDevice);
        ROS_INFO_STREAM("111");
        // defaulting some common auto-exposure settings
        getParamCheck(nh, "exposure_auto", acq_autoexp_mode, std::string("Off"));
        ROS_INFO_STREAM("exposure_auto = " << acq_autoexp_mode);
        cfg.acq_exposure_AECMode = acq_autoexp_mode;
        m_GenICamACQ_ptr->exposureAuto.writeS(std::string(cfg.acq_exposure_AECMode));
        
        //m_GenICamACQ_ptr->mvExposureAutoAverageGrey.writeS(std::string("50"));
        //m_GenICamACQ_ptr->acquisitionFrameRateEnable.write(bTrue);
        //m_GenICamACQ_ptr->acquisitionFrameRate.writeS(std::string("30"));
        //m_GenICamACQ_ptr->mvExposureAutoMode.writeS(std::string("mvDevice"));
        
        // Triggering
        getParamCheck(nh, "acquisition_mode", acq_mode, std::string("Continuous")); // SingleFrame
        ROS_INFO_STREAM("acquisition_mode = " << acq_mode);
        cfg.acq_mode = acq_mode;
        m_GenICamACQ_ptr->acquisitionMode.writeS(acq_mode);
        m_GenICamACQ_ptr->exposureTime.writeS(std::string("1000"));
        m_GenICamACQ_ptr->exposureMode.writeS(std::string("Timed"));
	m_GenICamACQ_ptr->acquisitionFrameRateEnable.write(bTrue);
        m_GenICamACQ_ptr->acquisitionFrameRate.writeS(std::string("20"));	
        getParamCheck(nh, "trigger_enable", acq_trigger_enable, std::string("Off"));
        ROS_INFO_STREAM("acq_trigger_enable = " << acq_trigger_enable);
        cfg.acq_trigger_mode = acq_trigger_enable;
        m_GenICamACQ_ptr->triggerMode.writeS( cfg.acq_trigger_mode );
        getParamCheck(nh, "trigger_select", acq_trigger_select, std::string("FrameStart"));
        ROS_INFO_STREAM("acq_trigger_select = " << acq_trigger_select);
        cfg.acq_trigger_selector = acq_trigger_select;
        m_GenICamACQ_ptr->triggerSelector.writeS( cfg.acq_trigger_selector );
        getParamCheck(nh, "trigger_source", acq_trigger_source, std::string("Line5"));//("Software"));  //Yoni
        ROS_INFO_STREAM("acq_trigger_source = " << acq_trigger_source);
        cfg.acq_trigger_source = acq_trigger_source;
        m_GenICamACQ_ptr->triggerSource.writeS( cfg.acq_trigger_source );
        getParamCheck(nh, "trigger_activation", acq_trigger_activation, std::string("AnyEdge"));
        ROS_INFO_STREAM("acq_trigger_activation = " << acq_trigger_activation);
        cfg.acq_trigger_activation = acq_trigger_activation;
        m_GenICamACQ_ptr->triggerActivation.writeS(std::string("AnyEdge"));//cfg.acq_trigger_activation );
        ROS_INFO_STREAM("222");               
        m_dynRecServer_ptr->updateConfig(cfg);

        // for possible future use
        m_imgProc_ptr = std::make_shared<ImageProcessing>(m_cameraDevice);

        /*m_GenICamCounterTimer_ptr = std::make_shared<GenICam::CounterAndTimerControl>(m_cameraDevice);
        m_GenICamCounterTimer_ptr -> counterSelector.writeS(std::string("Counter1"));
        m_GenICamCounterTimer_ptr -> counterEventSource.writeS(std::string("Line5"));
        m_GenICamCounterTimer_ptr -> counterEventActivation.writeS(std::string("RisingEdge"));
		    m_GenICamCounterTimer_ptr -> counterValue.writeS(std::string("0"));
        m_GenICamCounterTimer_ptr -> counterResetSource.writeS(std::string("Counter1End"));
        m_GenICamCounterTimer_ptr -> counterTriggerSource.writeS(std::string("Counter1End"));
        m_GenICamCounterTimer_ptr -> counterDuration.writeS(std::string("5"));*/
	ROS_INFO_STREAM("333"); 
        // Setup imageFormatControl configuration
        m_GenICamImageFormat_ptr = std::make_shared<GenICam::ImageFormatControl>(m_cameraDevice);
        m_GenICamImageFormat_ptr -> pixelFormat.writeS(cfg.ifc_pixel_format);
        m_GenICamImageFormat_ptr -> width.writeS(cfg.ifc_width);
        m_GenICamImageFormat_ptr -> height.writeS(cfg.ifc_height);        
        m_GenICamImageFormat_ptr -> offsetX.writeS(std::string("0")); //180// 320 is for width of 1280
        m_GenICamImageFormat_ptr -> offsetY.writeS(std::string("0")); // 136
        m_GenICamImageFormat_ptr -> reverseY.write(bFalse);
        ROS_INFO_STREAM("444"); 
	//m_GenICamImageFormat_ptr -> BinningHorizontalMode.writeS(std::string("Average"));
	m_GenICamImageFormat_ptr -> binningHorizontal.writeS(std::string("2"));
	//m_GenICamImageFormat_ptr -> BinningVerticalMode.writeS(std::string("Average"));
	m_GenICamImageFormat_ptr -> binningVertical.writeS(std::string("2"));
        //        
	// Setup ImageDestination pointer
        m_destinationFormat_ptr = std::make_shared<ImageDestination>(m_cameraDevice);
         ROS_INFO_STREAM("555"); 
        // Set up ChunkData configuration
        m_GenICamImageChunk_ptr = std::make_shared<GenICam::ChunkDataControl>(m_cameraDevice);
        m_GenICamImageChunk_ptr -> chunkModeActive.write(bTrue);
        m_GenICamImageChunk_ptr -> chunkSelector.writeS(std::string("ExposureTime"));
        m_GenICamImageChunk_ptr -> chunkEnable.write(bTrue);
        m_GenICamImageChunk_ptr -> chunkSelector.writeS(std::string("Timestamp"));
        m_GenICamImageChunk_ptr -> chunkEnable.write(bTrue);
         ROS_INFO_STREAM("666"); 
        // Setup AnalogControl configuration
        //m_GenICamAnalog_ptr = std::make_shared<GenICam::AnalogControl>(m_cameraDevice);
        //m_GenICamAnalog_ptr -> mvGammaEnable.write(bTrue);//S(std::string("On"));

        // image callback statistics pointer
        m_threadParam_ptr = std::make_shared<ThreadParameter>(m_cameraDevice);

        // Request pointer (see wxPropView 'Request' field for more info)
        requestProvider_ptr = std::make_shared<helper::RequestProvider>(m_cameraDevice);

        // setup dynamic reconfigure callback
        const auto cbk_dynRec = boost::bind(&Bluefox3::dynRecCallback, this, _1, _2);
        m_dynRecServer_ptr->setCallback(cbk_dynRec);

	
        // | ----------- Subscribe to trigger ----------- |
        // setup the trigger subscriber callback
        const auto cbk_sub = boost::bind(&Bluefox3::triggerCallback, this, _1);
        m_sub = _nh.subscribe<std_msgs::Header>("trigger", 10, cbk_sub); //trigger is published by user from console. example in page 4 of Omer_matrix.odt

        // | ----------- Start the actual image acquisition ----------- |
        // setup acquisition callback, and start the queue.
        const auto cbk_img = std::bind(&Bluefox3::imageCallback, this, std::placeholders::_1, std::placeholders::_2);
        requestProvider_ptr->acquisitionStart(cbk_img, m_threadParam_ptr);
        
        m_running = true;

        // | ----------------- Initialization complete ---------------- |
        ROS_INFO("[%s]: Initialized, acquisition started", m_node_name.c_str());
    }
        

	/* ~Bluefox3() destructor //{ */
    Bluefox3::~Bluefox3()
    {
      if (m_running)
        requestProvider_ptr->acquisitionStop();
      if (m_cameraDevice && m_cameraDevice->isOpen())
        m_cameraDevice->close();
    }
	

 }  // namespace bluefox3

    #include <pluginlib/class_list_macros.h>
    PLUGINLIB_EXPORT_CLASS(bluefox3::Bluefox3, nodelet::Nodelet)
