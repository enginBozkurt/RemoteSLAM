#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import pigpio
import time
import binascii
import numpy as np
from timeit import default_timer as timer


# Global Variables
MPU_9150_I2C_ADDRESS_1     =     0x69    # Base address of the Drotek board
MPU_9150_I2C_ADDRESS_2     =     0x68    # Base address of the SparkFun board
MPU_9150_SMPRT_DIV         =     0x19    # Gyro sampling rate divider
MPU_9150_DEFINE            =     0x1A    # Gyro and accel configuration
MPU_9150_GYRO_CONFIG       =     0x1B    # Gyroscope configuration
MPU_9150_ACCEL_CONFIG      =     0x1C    # Accelerometer configuration
MPU_9150_FIFO_EN           =     0x23    # FIFO buffer control
MPU_9150_INT_PIN_CFG       =     0x37    # Bypass enable configuration
MPU_9150_INT_ENABLE        =     0x38    # Interrupt control
MPU_9150_ACCEL_XOUT_H      =     0x3B    # Accel X axis High
MPU_9150_ACCEL_XOUT_L      =     0x3C    # Accel X axis Low
MPU_9150_ACCEL_YOUT_H      =     0x3D    # Accel Y axis High
MPU_9150_ACCEL_YOUT_L      =     0x3E    # Accel Y axis Low
MPU_9150_ACCEL_ZOUT_H      =     0x3F    # Accel Z axis High
MPU_9150_ACCEL_ZOUT_L      =     0x40    # Accel Z axis Low
MPU_9150_GYRO_XOUT_H       =     0x43    # Gyro X axis High
MPU_9150_GYRO_XOUT_L       =     0x44    # Gyro X axis Low
MPU_9150_GYRO_YOUT_H       =     0x45    # Gyro Y axis High
MPU_9150_GYRO_YOUT_L       =     0x46    # Gyro Y axis Low
MPU_9150_GYRO_ZOUT_H       =     0x47    # Gyro Z axis High
MPU_9150_GYRO_ZOUT_L       =     0x48    # Gyro Z axis Low
MPU_9150_USER_CTRL         =     0x6A    # User control
MPU_9150_PWR_MGMT_1        =     0x6B    # Power management 1

MPU_9150_I2C_MAGN_ADDRESS  =     0x0C    # Address of the magnetometer in bypass mode
MPU_9150_WIA               =     0x00    # Mag Who I Am
MPU_9150_AKM_ID            =     0x48    # Mag device ID
MPU_9150_ST1               =     0x02    # Magnetometer status 1
MPU_9150_HXL               =     0x03    # Mag X axis Low
MPU_9150_HXH               =     0x04    # Mag X axis High
MPU_9150_HYL               =     0x05    # Mag Y axis Low
MPU_9150_HYH               =     0x06    # Mag Y axis High
MPU_9150_HZL               =     0x07    # Mag Z axis Low
MPU_9150_HZH               =     0x08    # Mag Z axis High
MPU_9150_ST2               =     0x09    # Magnetometer status 2
MPU_9150_CNTL              =     0x0A    # Magnetometer control


IMUPI_BLOCK_SIZE           =     6	 # Accelerometer, Gyro block size
IMUPI_MES_SIZE             =     14	 # Total block size for single read
IMUPI_NB_AXIS              =     3
IMUPI_I2C_AUTO_INCREMENT   =     0x80
IMUPI_A_GAIN               =     6.103515625e-05  #  = 2/32768
IMUPI_G_GAIN               =     0.00762939453 #  = 250/32768
IMUPI_M_GAIN               =     0.3001221001221001
IMUPI_GOFF_NB_ITER         =     500

IMUPI_NO_ERROR             =     0
IMUPI_I2C_OPEN_ERROR       =     1
IMUPI_I2C_DEV_NOT_FOUND    =     2
IMUPI_I2C_WRITE_ERROR      =     3
IMUPI_I2C_READ_ERROR       =     4
IMUPI_INIT_ERROR           =     5

M_PI                       =     3.14159265358979323846
M_G                        =     9.81

GYRO_OFF_X = 0
GYRO_OFF_Y = 0
GYRO_OFF_Z = 0

mpu_seq = 0
trig_seq = 0
init_time = False
read_time = 0
td = - rospy.Duration.from_sec(0.002)  # Accel latency is 2ms, gyro is 1.9ms
trig_td = rospy.Duration.from_sec(0.001)  

def init_mpu9150(pigpio_ptr):

	h = pigpio_ptr.i2c_open(1, MPU_9150_I2C_ADDRESS_2)
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_PWR_MGMT_1, 0x00) #Put device in sleep mode
	time.sleep(0.1)
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_PWR_MGMT_1, 0x01) #Put device in sleep mode
	time.sleep(0.1)
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_SMPRT_DIV, 0x04)  #100Hz sample rate (1kHz/(1+divider)
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_DEFINE, 0x03)  #No ext sync, minimal latency low pass filter
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_ACCEL_CONFIG, 0x00)  #Accel range
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_GYRO_CONFIG, 0x00)  #Gyro range
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_FIFO_EN, 0x00)  #Disable FIFO
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_INT_PIN_CFG, 0x02)  #Bypass mode enabled 
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_USER_CTRL, 0x00)  #No FIFO and no I2C slaves
	pigpio_ptr.i2c_write_byte_data(h, MPU_9150_INT_ENABLE, 0x00)  #Enable interrupts
	#pigpio_ptr.i2c_write_byte_data(h, MPU_9150_PWR_MGMT_1, 0x00)  #No power management, internal clock source

	#sleep for a second to let device initialize
	#time.sleep(2)

        return h

def read_accel_data(pigpio_ptr, handle):
        data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_ACCEL_XOUT_H, IMUPI_BLOCK_SIZE)[1]


def read_gyro_data(pigpio_ptr, handle):
        data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_GYRO_XOUT_H, IMUPI_BLOCK_SIZE)[1]

def read_data(pigpio_ptr, handle):
	global init_time, read_time, td
	data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_ACCEL_XOUT_H, IMUPI_MES_SIZE)[1]
        while(data == ''):
		print("EMPTY")
		data = pigpio_ptr.i2c_read_i2c_block_data(handle, MPU_9150_ACCEL_XOUT_H, IMUPI_MES_SIZE)[1]
	
        
	return data, read_time #+ td

'''
IMUPI_A_GAIN               =     6.103515625e-05   //  = 2/32768
IMUPI_G_GAIN               =     0.030487804878049 //  = 250/32768
'''

def decode_regs_encode_msg(data, header=None, calibration=False):
	global td

	ax = np.int16(int(binascii.hexlify(data[0:2]), 16)) * IMUPI_A_GAIN * M_G
	ay = np.int16(int(binascii.hexlify(data[2:4]), 16)) * IMUPI_A_GAIN * M_G
	az = np.int16(int(binascii.hexlify(data[4:6]), 16)) * IMUPI_A_GAIN * M_G

	gx = np.int16(int(binascii.hexlify(data[8:10]), 16)) * IMUPI_G_GAIN * 0.0174533 #- GYRO_OFF_X  # 0.0174533 = pi/180
	gy = np.int16(int(binascii.hexlify(data[10:12]), 16)) * IMUPI_G_GAIN * 0.0174533 #- GYRO_OFF_Y
	gz = np.int16(int(binascii.hexlify(data[12:14]), 16)) * IMUPI_G_GAIN * 0.0174533 #- GYRO_OFF_Z
	
	if calibration == False:
		msg = Imu()
		msg.linear_acceleration.x = ax
		msg.linear_acceleration.y = ay
		msg.linear_acceleration.z = az

		msg.angular_velocity.x = gx
		msg.angular_velocity.y = gy
		msg.angular_velocity.z = gz

		msg.header = header
		#msg.header.stamp = msg.header.stamp + td
		return msg

	else:
		#print('ax: {0},  ay: {1},  az: {2},	gx: {3},  gy: {4},  gz: {5}'.format(ax, ay, az, gx, gy, gz))
		return ax, ay, az, gx, gy, gz

def calc_gyro_offset():
	print("Calculating Gyro Offsets...")
	_off1 = 0
	_off2 = 0
	_off3 = 0
	for i in range(IMUPI_GOFF_NB_ITER):
		data, _ = read_data(pi, h)
		res=decode_regs_encode_msg(data, calibration=True)
		_off1 += res[3]
		_off2 += res[4]
		_off3 += res[5]
	
	GYRO_OFF_X = _off1/IMUPI_GOFF_NB_ITER
	GYRO_OFF_Y = _off2/IMUPI_GOFF_NB_ITER
	GYRO_OFF_Z = _off3/IMUPI_GOFF_NB_ITER
	print("X: {0},		Y: {1},		Z: {2}".format(GYRO_OFF_X, GYRO_OFF_Y, GYRO_OFF_Z))

'''
def cbf(gpio, level, tick):
	global mpu_seq, trig_seq
	print("cbf")
	data, corrected_stamp = read_data(pi, h)
	#print(data)
	mh = Header()
	mh.stamp = corrected_stamp
	mh.frame_id = 'mpu9150_frame'
	mh.seq = mpu_seq
	res=decode_regs_encode_msg(data, header=mh)
	#print("imu stamp {0}: {1}".format(res.header.seq, res.header.stamp))
	#print("cbk time: {0}".format(rospy.Time.now()))
	pub.publish(res)
	if (trig_flag and mpu_seq != 0 and mpu_seq % trig_div == 1):
		#pi.gpio_trigger(gpio_trig_num, 100, 1)
		tmp = Header()
		tmp.stamp = corrected_stamp + 4 * trig_td
		tmp.seq = trig_seq
		tmp.frame_id = 'mpu9150_frame'
		#print("stamp {0}: {1}s  :  {2}ns".format(trig_seq + 1, tmp.stamp.secs, tmp.stamp.nsecs))
		pub_trig.publish(tmp)
		trig_seq += 1
	
	mpu_seq += 1
'''


pub = rospy.Publisher('/imu', Imu, queue_size=10)
#pub_trig = rospy.Publisher('/trigger', Header, queue_size = 10)
rospy.init_node('mpu9150', anonymous=True)

full_param_name = rospy.search_param('host_sample_rate')
if full_param_name == None:
	full_param_name = 'host_sample_rate'
read_rate = rospy.get_param(full_param_name, 100)
rate = rospy.Rate(read_rate)
#print(read_rate)
print("read_rate: {0} Hz".format(read_rate))

full_param_name = rospy.search_param('trigger_enable')
if full_param_name == None:
	full_param_name = 'trigger_enable'
trigger_enable = rospy.get_param(full_param_name, True)
trig_flag = trigger_enable
print("trigger_enable: {0} ".format(trigger_enable))

full_param_name = rospy.search_param('trigger_divider')
if full_param_name == None:
	full_param_name = 'trigger_divider'
trigger_divider = rospy.get_param(full_param_name, 5)
trig_div = trigger_divider
print("trig_div: {0} ".format(trig_div))

full_param_name = rospy.search_param('gpio_num')
if full_param_name == None:
	full_param_name = 'gpio_num'
gpio_num = rospy.get_param(full_param_name, 23)
gpio_trig_num = gpio_num
print("gpio_trig_num: {0} ".format(gpio_num))


pi = pigpio.pi()

curr_value = 0

if trig_flag:
	pi.set_mode(gpio_trig_num, pigpio.OUTPUT)
	pi.write(gpio_trig_num, 0)
	pi.set_pull_up_down(gpio_trig_num, pigpio.PUD_DOWN)

#cb1 = pi.callback(24, pigpio.RISING_EDGE, cbf)

h = init_mpu9150(pi)

#calc_gyro_offset()
time.sleep(0.4)


read_time_prev = rospy.Time.now()
#rate = rospy.Rate(100)

while not rospy.is_shutdown():
    before = timer()
    read_time__ = rospy.Time.now()
    data, _ = read_data(pi, h)
    after = timer()
    total = after - before
    #print(total)
    if (total > 0.01): #150 hz is 0.0066666
        print("total reading imu time: {0} sec".format(total))

    mh = Header()
    #total = rospy.Duration.from_sec(total/2)
    mh.stamp = read_time__ #+ total
    mh.frame_id = 'mpu9150_frame'
    mh.seq = mpu_seq


    '''if (mpu_seq % 5 == 0):
        if (curr_value==1):
            curr_value=0
        else:
            curr_value = 1

        pi.write(gpio_trig_num, curr_value)
        temp_time = read_time__.to_sec()-read_time_prev.to_sec()
        if ((temp_time>0.06)):
            rospy.loginfo(str(temp_time))
            #rospy.loginfo( str(read_time__.to_sec()))
            #|(temp_time<0.033)):
            
        read_time_prev = read_time__
	#rospy.loginfo( str(read_time__.to_sec()))'''
       
    

    res = decode_regs_encode_msg(data, header=mh)
    pub.publish(res)
    mpu_seq += 1
    rate.sleep()


pi.i2c_write_byte_data(h, MPU_9150_INT_ENABLE, 0x00)  #Disable interrupts
print("Here 2")
time.sleep(2)



