# RemoteSLAM
Provide  building blocks (software, hardware and algorithms) for implementing SLAM using small sensors

## The purpose of this repo is to provide the building blocks (software, hardware and algorithms) for implementing SLAM using a camera and an IMU.
We plan to improve it as we go, while receiving assistance from the community.
Currently the tool supports 3 models of monochrome cameras and the Invense MPU
UI3271LE-M-GL-VU
https://en.ids-imaging.com/store/ui-3271le-vu.html

mvBlueFOX3-1013GE
https://www.matrix-vision.com/USB3-vision-single-board-camera-mvbluefox3-m.html

https://www.waveshare.com/imx219-200-camera.htm

You can choose a specific lense but fisheye, or wide FOV, lense is reccomended  for a better __
Currently, the heavy algorithms run on a host computer, receiving sensor data over wifi. This makes it easier to test and develop different SLAM algorithms and new features.
The SLAM algorithms currently supported are:
OrbSLAM3 (mono and mono+imu)
VINS-Fusion (mono+imu)
DSO and LDSO (mono)
ROS wrappers (for real time implementations) are provided for both the sensors and the algorithms.

Installations:
Pi You can download an Ubuntu18 based image for the pi4 here.

Host ORBSLAM3 can be installed from


Operating instructions:
On the pi, the ROS workspace for the Bluefox camera is named mvBluefox_ws. Remember to source it in the .bashrc . the ROS workspace for the IDS camera is named
