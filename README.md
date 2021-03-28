# RemoteSLAM

The purpose of this repo is to provide the building blocks (software, hardware and algorithms) for implementing SLAM using small and lite weight sensors (e.g., cameras, imu).
We plan to improve it as we go, while receiving assistance from the community.
Currently the tool supports 3 types of monochrome cameras and the Invense MPU

UI3271LE-M-GL https://en.ids-imaging.com/store/ui-3271le-vu.html

mvBlueFOX3-1013GE https://www.matrix-vision.com/USB3-vision-single-board-camera-mvbluefox3-m.html

https://www.waveshare.com/imx219-200-camera.htm

For the lense, a fisheye or wide FOV lense, is highly reccomended.
Currently, the heavy lifting (SLAM algorithms) takes place on a remote host computer, receiving sensor data over wifi. This makes it easier both to test and develop different SLAM algorithms and new features.

The SLAM algorithms currently supported are:

OrbSLAM3 (mono and mono+imu)

VINS-Fusion (mono+imu)

DSO and LDSO (mono)

ROS wrappers (for real time implementations) are provided for both the sensors and the SLAM algorithms.

#
## Hardware and software overview

![PI_Case](https://github.com/tau-adl/RemoteSLAM/blob/main/PI_case_small.jpg)

A Raspberry Pi 4, running Ubuntu 18, was chosen as the the edge computer for reading and transmitting the sensors' data, over wifi. General installation instructions for the PI are provided here                   . A PI image will be uploaded in the near future.

An ubuntu 18 host computer receives the data from the PI, and runs the different SLAM algorithms. We try to provide all the links, relevant information and Ros wrappers needed  for implementing and running the different SLAM algorithms in real time. 
