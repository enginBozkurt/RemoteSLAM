# Bluefox3 ROS driver

## Description

A ROS driver for the mvbluefox3 for the rpi4 model-b (4gb). This driver is coupled with the bno055/mpu9150 IMU's drivers from TAU-adl to present a hardware synchronized sensor configuration for visual-intertial slam. 

*Features so far:*
* image and camera info publication as ROS messages
* acquisition configuration
* resolution configuration
* device pixel format
* readout (and ROS msg) pixel format
* trigger enable/disable, source and activation mode*
* manual, constant exposure time configuration*
* autoexposure mode configuration, including minimum and maximum exposure time*

(* = Dynamically reconfigurable parameters. Other parameters are set at launch.)
To set parameters dynamically, run rosrun rqt_reconfigure rqt_reconfigure

## Prequisites

The `mvIMPACT_Acquire` library corresponding to your camera is required to be installed and in present path (must be findable by CMake, installation instructions: https://www.matrix-vision.com/manuals/mvBlueFOX3/mvBC_page_quickstart.html#mvBC_subsubsection_quickstart_linux_requirements_installing).
C++ standard >= `c++11` is required for compilation.
Tested with ROS Melodic, but should work with older/newer versions as well.

`pigpio` http://abyz.me.uk/rpi/pigpio/index.html

## Parameters
# Mandatory
**~camera_serial** (string)
* The camera serial as extracted from list-cameras. This is a mandatory parameter, to be set as 'serial:=XXXX' on launch.

**~camera_name** (string)
* The camera name. This is a mandatory parameter, and set to default of 'bluefox3_XXXXX' (serial) on launch.

**~frame_id** (string)
* The node frame_id. This is a mandatory parameter, and set to default of camera name on launch.

**~calib_url** (string)
* YAML calibration file URL. Currently not used.

# Optional
**~acquisition_mode** (string, default: Continuous)
* sets acquisiotion mode to 'Continuous' - video capture.
* other possible values are 'SingleFrame' - still capture, and 'MultipleFrame' - burst capture (unused).

**~exposure_time** (string, default: 10000)
* sets manual exposure time if AEC mode is off.
* reasonable values are from 5000 - 60000.

**~exposure_auto** (string, default: Continuous)
* sets AEC mode to 'Continuous' - the exposure is calculated online, frame-to-frame.
* other possible values are 'Once' - calculate on initialization, and 'Off' - manual exposure time.

**~exposure_auto_upper_limit** (double, default: 25000.0)
* sets maximal exposure time if AEC mode is on.
* reasonable values are from 15000 - 60000.

**~exposure_auto_lower_limit** (double, default: 10000.0)
* sets minimal exposure time if AEC mode is on.
* reasonable values are from 5000 - 15000.

**~exposure_auto** (string, default: Continuous)
* sets AEC mode to 'Continuous - the exposure is calculated online, frame-to-frame.
* other possible values are 'Once' - calculate on initialization, and 'Off' - manual exposure time.

**Triggering Parameters**
**~trigger_enable** (string, default: On)
* Camera trigger functionality enabled.
* Other possible values are 'Off' - camera trigger funcionality disabled.

**~trigger_select** (string, default: FrameStart)
* Camera trigger functionality starts frame acquisition.
* For other possible values see wxPropView.

**~trigger_source** (string, default: Software)
* Camera trigger functionality starts on Software call.
* For other possible values see wxPropView.

**~trigger_activation** (string, default: AnyEdge)
* If camera trigger functionality's source is an I/O line, set it to trigger on rising and falling edge.
* For other possible values see wxPropView.

## Acknowledgements
Originally forked from https://github.com/ctu-mrs/bluefox3_ros
The code was inspired by the KumarRobotics driver for bluefox2: https://github.com/KumarRobotics/bluefox2.
