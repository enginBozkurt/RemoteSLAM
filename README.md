# RemoteSLAM

The purpose of this repo is to provide the building blocks (software, hardware and algorithms) for implementing SLAM systems using small and lite weight sensors (e.g., cameras, imu).
We plan to improve this repo as we go, while receiving assistance from the community.
Currently, the tool supports 3 types of monochrome cameras/sensors, combined with the Invense MPU 9X50 family:
* Matrix-Vision:  mvBlueFOX3-M/MLC https://www.matrix-vision.com/USB3-vision-single-board-camera-mvbluefox3-m.html
* IDS-Imaging:  UI3271LE-M-GL   https://en.ids-imaging.com/store/ui-3271le-vu.html
* Arducam:   https://www.arducam.com/products/camera-breakout-board/global-shutter-camera/
* Waveshare: https://www.waveshare.com/imx219-200-camera.htm

For the lense, a fisheye or wide FOV lense is highly recommended.
Currently, the heavy lifting (SLAM algorithms) takes place on a remote host computer, receiving sensor data over wifi. This makes it easier both to test and develop different SLAM algorithms and new features.

The SLAM algorithms currently supported are:
* OrbSLAM3 (mono and mono+imu)
* VINS-Fusion (mono+imu)
* DSO and LDSO (mono)

ROS wrappers (for real time implementations) are provided for both the sensors and the SLAM algorithms.


**Please find more information on the [wiki pages](https://github.com/tau-adl/RemoteSLAM/wiki) of this repository.**

**For questions or comments, please open an issue on Github.**


#
#
## Hardware and software overview

![PI_Case](https://github.com/tau-adl/RemoteSLAM/blob/main/PI_case_small.jpg)

A Raspberry Pi 4, running Ubuntu 18, was chosen as the the edge computer for reading and transmitting the sensors' data, over wifi. General installation instructions for the PI are provided here                   . A PI image will be uploaded in the near future.

An ubuntu 18 host computer receives the data from the PI, and runs the different SLAM algorithms. We try to provide all the links, relevant information and Ros wrappers needed  for implementing and running the different SLAM algorithms in real time. 


##Authors

##References




 is a toolbox that solves the following  problems:

1. **Multiple camera **: 
    intrinsic a
1. **Visual-inertial (camera-IMU)**:
    spatial and t
1. **Rolling Shutter **:
    full intrinsic 
