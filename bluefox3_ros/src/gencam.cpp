#include <iostream>
#include <ios>
#include <memory>
#include <sstream>
#include <thread>
#include <chrono>
#include <thread>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_helper.h>
#ifdef _WIN32
#   include <mvDisplay/Include/mvIMPACT_acquire_display.h>
using namespace mvIMPACT::acquire::display;
#   define USE_DISPLAY
#else
#   include <stdio.h>
#   include <unistd.h>
#endif // #ifdef _WIN32

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;
using namespace std;

image_transport::CameraPublisher m_pub;

//=============================================================================
//================= Data type definitions =====================================
//=============================================================================
//-----------------------------------------------------------------------------
struct ThreadParameter
//-----------------------------------------------------------------------------
{
    Device* pDev_;
    unsigned int requestsCaptured_;
    Statistics statistics_;
    mvIMPACT::acquire::GenICam::DigitalIOControl dio_;
#ifdef USE_DISPLAY
    ImageDisplayWindow displayWindow_;
#endif // #ifdef USE_DISPLAY
    explicit ThreadParameter( Device* pDev, mvIMPACT::acquire::GenICam::DigitalIOControl& dio ) : pDev_( pDev ), requestsCaptured_( 0 ), statistics_( pDev ), dio_( dio )
#ifdef USE_DISPLAY
    // initialise display window
        // IMPORTANT: It's NOT safe to create multiple display windows in multiple threads!!!
        , displayWindow_( "mvIMPACT_acquire sample, Device " + pDev_->serial.read() )
#endif // #ifdef USE_DISPLAY
    {}
    ThreadParameter( const ThreadParameter& src ) = delete;
    ThreadParameter& operator=( const ThreadParameter& rhs ) = delete;
};

//=============================================================================
//================= implementation ============================================
//=============================================================================

//-----------------------------------------------------------------------------
static bool canRestoreFactoryDefault( Device* pDev )
//-----------------------------------------------------------------------------
{
    GenICam::UserSetControl usc( pDev );
    if( !usc.userSetSelector.isValid() || !usc.userSetLoad.isValid() )
    {
        return false;
    }
    vector<string> validUserSetSelectorStrings;
    usc.userSetSelector.getTranslationDictStrings( validUserSetSelectorStrings );
    return find( validUserSetSelectorStrings.begin(), validUserSetSelectorStrings.end(), "Default" ) != validUserSetSelectorStrings.end();
}

//-----------------------------------------------------------------------------
void myThreadCallback( shared_ptr<Request> pRequest, ThreadParameter& threadParameter )
//-----------------------------------------------------------------------------
{
    ++threadParameter.requestsCaptured_;
    // display some statistical information every 100th image
    if( threadParameter.requestsCaptured_ % 100 == 0 )
    {
        const Statistics& s = threadParameter.statistics_;
        cout << "Info from " << threadParameter.pDev_->serial.read()
             << ": " << s.framesPerSecond.name() << ": " << s.framesPerSecond.readS()
             << ", " << s.errorCount.name() << ": " << s.errorCount.readS()
             << ", " << s.captureTime_s.name() << ": " << s.captureTime_s.readS()
             << ", LineStatusAll: " << threadParameter.dio_.lineStatusAll.read() << endl;
    }
    if( pRequest->isOK() )
    {
        cout << "Image captured: " << pRequest->imageOffsetX.read() << "x" << pRequest->imageOffsetY.read() << "@" << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << endl;
        const auto imw = pRequest->imageWidth.read();
        const auto imh = pRequest->imageHeight.read();
        const auto imdata = pRequest->imageData.read();
        const auto imstep = pRequest->imageLinePitch.read();

        sensor_msgs::Image image_msg;
        sensor_msgs::fillImage(image_msg, sensor_msgs::image_encodings::BAYER_GRBG8, imh, imw, imstep, imdata);
        sensor_msgs::CameraInfo cinfo_msg;
        m_pub.publish(image_msg, cinfo_msg);
    }
    else
    {
        cout << "Error: " << pRequest->requestResult.readS() << endl;
    }
}

//-----------------------------------------------------------------------------
// This function will allow to select devices that support the GenICam interface
// layout(these are devices, that claim to be compliant with the GenICam standard)
// and that are bound to drivers that support the user controlled start and stop
// of the internal acquisition engine. Other devices will not be listed for
// selection as the code of the example relies on these features in the code.
bool isDeviceSupportedBySample( const Device* const pDev )
//-----------------------------------------------------------------------------
{
    if( !pDev->interfaceLayout.isValid() &&
        !pDev->acquisitionStartStopBehaviour.isValid() )
    {
        return false;
    }

    vector<TDeviceInterfaceLayout> availableInterfaceLayouts;
    pDev->interfaceLayout.getTranslationDictValues( availableInterfaceLayouts );
    return find( availableInterfaceLayouts.begin(), availableInterfaceLayouts.end(), dilGenICam ) != availableInterfaceLayouts.end();
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
//-----------------------------------------------------------------------------
{

    ros::init(argc, argv, "bluefox3");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    m_pub = it.advertiseCamera("image_raw", 5);

    DeviceManager devMgr;
    cout << "--------------------------------------------!!! ATTENTION !!!--------------------------------------------" << endl;
    cout << "Please be aware that the digital outputs of the device might be enabled during the test." << endl
         << "This might lead to unexpected behavior in case of devices which are connected to one of the digital outputs," << endl
         << "so only proceed if you are sure that this will not cause any issue with connected hardware!!" << endl;
    cout << "---------------------------------------------------------------------------------------------------------" << endl;
    cout << "" << endl;

    Device* pDev = getDeviceFromUserInput( devMgr, isDeviceSupportedBySample );
    if( pDev == nullptr )
    {
        cout << "Unable to continue! Press [ENTER] to end the application" << endl;
        cin.get();
        return 1;
    }

    cout << "Initialising the device. This might take some time..." << endl;
    cout << endl;
    try
    {
        pDev->interfaceLayout.write( dilGenICam ); // This is also done 'silently' by the 'getDeviceFromUserInput' function but your application needs to do this as well so state this here clearly!
        pDev->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening the device " << pDev->serial.read()
             << "(error code: " << e.getErrorCodeAsString() << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 1;
    }

    // To make sure the device will be configured based on a defined state, the default user set will be loaded
    mvIMPACT::acquire::GenICam::UserSetControl usc( pDev );

    cout << "The device will be configured now!\n" << endl;

    // Make sure the default user set and the load method exist
    // In general it is a good idea to verify if the GenICam class which should be used exists and is of the expected type
    if( canRestoreFactoryDefault( pDev ) )
    {
        if( usc.userSetSelector.isValid() && usc.userSetSelector.isWriteable() && usc.userSetLoad.isValid() && usc.userSetLoad.isMeth() )
        {
            cout << "Loading the device's default user set to avoid undefined settings!\n" << endl;
            // selecting the default user set which includes the factory settings of the device
            usc.userSetSelector.writeS( "Default" );
            // call the userSetLoad method to load the currently selected user set
            usc.userSetLoad.call();
        }
    }
    else
    {
        cout << "The device seems not to support the default user set!" << endl;
    }

    const double dExposureTime = 500;
    // initialise the AcquisitionControl class to get access to the exposure time property
    mvIMPACT::acquire::GenICam::AcquisitionControl acq( pDev );
    // Make sure the exposure time property does exist and is currently not "read only"
    if( acq.exposureTime.isValid() && acq.exposureTime.isWriteable() )
    {
        cout << "Currently the exposure time is set to " << acq.exposureTime.read() << " us. Changing to " << dExposureTime << " us" << endl;
        cout << endl;
        acq.exposureTime.write( dExposureTime );
    }

    // initialising the ImageFormatControl class to get access to the sensor's AOI settings
    mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );
    mvIMPACT::acquire::ImageDestination dest ( pDev );
    dest.pixelFormat.writeS("Mono8");
    ifc.pixelFormat.writeS("BayerGR8");

    // check if the sensors properties are available and modifying the AOI settings if width and height are valid properties
    if( ifc.width.isValid() && ifc.height.isValid() )
    {
        cout << "The sensor has a max resolution of about " << ifc.width.getMaxValue() << "x" << ifc.height.getMaxValue() << " pixels" << endl;
        cout << "The resolution will now be adjusted to the half of width and height. The resulting AOI will be: " << ifc.width.getMaxValue() / 2 << "x" << ifc.height.getMaxValue() / 2 << " pixels" << endl;
        // the AOI settings are usually not writable once the senor is exposing images so make sure width and height are not read-only at the moment
        if( !ifc.width.isWriteable() || !ifc.height.isWriteable() )
        {
            cout << "Width or Height are not writable at the moment." << endl;
        }
        else
        {
            ifc.width.write( ifc.width.getMaxValue() / 2 );
            ifc.width.write( ifc.height.getMaxValue() / 2 );
        }
    }

    mvIMPACT::acquire::GenICam::CounterAndTimerControl ctc( pDev );
    // Since the device's settings have a huge impact on the frame rate of the sensor, we need the device almost configured at this step. Otherwise the max frame rate would not be correct.
    cout << endl;
    cout << "To avoid some cabling work, we will use an internal timer for triggering in this sample!" << endl;
    cout << "The trigger frequency will be configured to half of the max frequency the sensor would be capable of in your setup." << endl;

    // Figuring out how many timers are available
    vector<string> availableTimers;
    if( ctc.timerSelector.isValid() )
    {
        ctc.timerSelector.getTranslationDictStrings( availableTimers );
    }
    if( ctc.timerSelector.isValid() && ctc.timerSelector.isWriteable() && ( availableTimers.size() >= 2 ) && acq.triggerSelector.isValid() && acq.mvResultingFrameRate.isValid() )
    {
        // Making sure that Timer2End as TimerTriggerSource does exist
        ctc.timerSelector.writeS( "Timer1" );
        vector<string> availableTriggerSources;
        ctc.timerTriggerSource.getTranslationDictStrings( availableTriggerSources );
        if( find( availableTriggerSources.begin(), availableTriggerSources.end(), "Timer2End" ) != availableTriggerSources.end() && acq.mvResultingFrameRate.isValid() )
        {
            const double dPeriod = 1000000. / ( acq.mvResultingFrameRate.read() / 2. );
            if( dPeriod >= 300. )
            {
                // Defining the duration the trigger signal is "low". The timer selector has not to be changed since it has been set to Timer1 already
                ctc.timerDuration.write( 1000. );
                ctc.timerTriggerSource.writeS( "Timer2End" );

                // Defining the duration the trigger signal is "high"
                ctc.timerTriggerSource.writeS( "Timer1End" );
                ctc.timerSelector.writeS( "Timer2" );
                ctc.timerDuration.write( dPeriod - 1000. );

                // Configuring the FrameStart trigger to use the start signal of Timer1 and enabling the trigger mode
                acq.triggerSelector.writeS( "FrameStart" );
                //acq.triggerSource.writeS( "Timer1Start" );
                acq.triggerSource.writeS( "Software" );
                acq.triggerMode.writeS( "On" );
                acq.exposureAuto.writeS("Off");
                acq.exposureTime.writeS("500");
            }
        }
        else
        {
            cout << "This device does not support expected timer trigger sources! The device will work in free run mode instead!" << endl;
        }
    }
    else
    {
        cout << "This device does not support timers! The device will work in free run mode instead!" << endl;
    }

    mvIMPACT::acquire::GenICam::AnalogControl anc( pDev );
    // Applying some gain to the signal provided by the device's sensor
    if( anc.gain.isValid() && anc.gain.isWriteable() )
    {
        anc.gain.write( anc.gain.getMaxValue() );
    }

    mvIMPACT::acquire::GenICam::DigitalIOControl dio( pDev );
    // Figuring out how many digital IOs are available
    cout << "\nAvailable Digital IOs:" << endl;
    vector<string> availableIOs;
    dio.lineSelector.getTranslationDictStrings( availableIOs );
    bool boConfiguredFirstOutput = false;

    // iterating over the vector of digital IOs to read out the lineMode, lineStatus and lineSource properties
    for( auto& line : availableIOs )
    {
        dio.lineSelector.writeS( line );
        // using mvExposureAndAcquisitionActive as the lineSource for the first digital output found
        if( !boConfiguredFirstOutput && dio.lineMode.readS() == "Output" )
        {
            dio.lineSource.writeS( "mvExposureAndAcquisitionActive" );
            boConfiguredFirstOutput = true;
        }
        cout << line << " - " << ( dio.lineMode.isValid() ? dio.lineMode.readS() : string( "UNSUPPORTED" ) )
             << " - LineStatus: " << ( dio.lineStatus.isValid() ? dio.lineStatus.readS() : string( "UNSUPPORTED" ) )
             << " - LineSource: " << ( dio.lineSource.isValid() ? dio.lineSource.readS() : string( "UNSUPPORTED" ) )
             << endl;
    }

    // start the execution of the 'live' thread.
    cout << "Press [ENTER] to end the application" << endl;
    ThreadParameter threadParam( pDev, dio );
    mvIMPACT::acquire::helper::RequestProvider requestProvider( pDev );
    requestProvider.acquisitionStart( myThreadCallback, std::ref( threadParam ) );
    while (true) {
        acq.triggerSoftware.call();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    cin.get();
    requestProvider.acquisitionStop();
    return 0;
}
