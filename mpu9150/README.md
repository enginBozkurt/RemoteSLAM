# MPU9150 PI
A ROS driver of MPU9150

## Device
MPU9150, 9250 or 6050

## Dependencies
pigpio http://abyz.me.uk/rpi/pigpio/index.html

## Published Topics
**imu9150** (sensor_msgs/Imu)
* discluding pose and covariance matrices
**trigger** (std_msgs/Header)
* trigger times, sequence

## Parameters

**~host_sample_rate** (int, default: 200)
* the IMU is sampling at a fixed rate of 1kHz. This parameter sets the read-out rate.

**Triggering Parameters. To work with triggering, first make sure pigpiod is running (sudo pigpiod).**

**~trigger_enable** (bool, default: true)
* enable gpio pulse triggering functionality.

**~trigger_divider** (int, default: 10)
* triggering will occur every host_sample_rate/trigger_divider imu data reads.

**~gpio_num** (int, default: 23)
* trigger out pin.



