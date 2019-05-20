#!/usr/bin/env python
# Simple Adafruit BNO055 sensor reading example.  Will print the orientation
# and calibration data every second.
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import rospy
import logging
import sys
import time
import math
import os
import signal
import serial
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from bosch_imu_emilk.msg import BOSCH_IMU_DATA
from Adafruit_BNO055 import BNO055


# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=18)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
a,b,c,d,e,f=bno.get_axis_remap()
print('Xaxis={0:0} Yaxis={1:0} Zaxis={2:0} Xsign={3:0} Ysign={4:0} Zsign={5:0}'.format(a,b,c,d,e,f))

# INITIALIZE PUBLISHERS
#pub_theta = rospy.Publisher('theta_gyro', Float32MultiArray, queue_size=1)
#pub_gyrospeed = rospy.Publisher('angular_speed', Float32, queue_size=1)
#pub_omega = rospy.Publisher('speed_command', Float32, queue_size=1)
#pub_accel= rospy.Publisher('x_accelerometer_data', Float32, queue_size=1)
#pub_linvelocity= rospy.Publisher('linear_velocity', Float32, queue_size=1)
pub_imu = rospy.Publisher('IMU_data', BOSCH_IMU_DATA, queue_size=1)
pub_theta = rospy.Publisher('IMU_theta', Float64, queue_size=1)
pub_rotspeed = rospy.Publisher('IMU_rotspeed', Float64, queue_size=1)
rospy.init_node('IMU_publisher', anonymous=False)
rate = rospy.Rate(100)   # 100 Hz sampling/publishing frequency 

def parse_input():
    data_end = False
    with open('/home/pi/ros_catkin_ws/src/bosch_imu_emilk/src/Adafruit_Python_BNO055/examples/imu_calib_data', 'r') as f:
        b = [0 for i in range(22)]
        count = 0
        i = 0
        while (not data_end):
            f.seek(i)
            temp = f.read(1)
            # print i, data_end, temp
            if temp == '[':
                count = 0
            elif temp == ']':
                data_end = True
            elif temp == ' ':
                c = 0
            elif temp == ',':
                count += 1
            else:
                try:
                    int(temp)
                except(ValueError):
                    print "file entry is not int"
                    break
                b[count] = 10*b[count] + int(temp)

            i += 1
    print b
    return b


def readIMU():

    i=0		#counter added for IMU calibration
    full_turns = 0
    prev_heading, _, _ = bno.read_euler()
    
    #headings = Float32MultiArray()
    data = BOSCH_IMU_DATA() 
    while not rospy.is_shutdown():      # The loop samples, CALCULATES AND PUBLISHES
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        #heading, roll, pitch = bno.read_euler()
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z = bno.read_linear_acceleration()
        # Gyroscope data (in degrees per second):
        data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z = bno.read_gyroscope()
        # Gyroscope data (in degrees per second):
        heading,roll,pitch = bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading,roll,pitch, sys, gyro, accel, mag))
        #value=mymessages()
	if (i==300):
	    bno.set_calibration(parse_input())

        if (prev_heading > 270) and (heading < 90):
            full_turns = full_turns + 1
        if (prev_heading < 90) and (heading > 270):
            full_turns = full_turns - 1
        prev_heading = heading
        heading = full_turns * 360 + heading

	#headings.data = [full_turns, heading]
	data.full_turns = full_turns
	data.euler_angles.z, data.euler_angles.y, data.euler_angles.x = heading, pitch, roll
	pub_theta.publish(heading)
        #pub_gyrospeed.publish(z)
        pub_rotspeed.publish(data.angular_velocity.z)
	pub_imu.publish(data)
	i = i + 1
        rate.sleep()


if __name__ == '__main__':
    readIMU()
