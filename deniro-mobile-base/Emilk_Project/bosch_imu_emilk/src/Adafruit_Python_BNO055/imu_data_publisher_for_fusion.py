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
from sensor_msgs.msg import Imu
import tf

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
pub = rospy.Publisher('IMU_msg', Imu, queue_size=1)
rospy.init_node('IMU_publisher', anonymous=False)
rate = rospy.Rate(100)   # 100 Hz sampling/publishing frequency 

def stdDevImu(orientation):
    global stdDevImu_init
    INPUT_SIZE = 6000
    if stdDevImu_init == False:
        # # initialisation
        stdDevImu.input_data = [[0.0 for i in range(INPUT_SIZE)], [0.0 for i in range(INPUT_SIZE)], [0.0 for i in range(INPUT_SIZE)]]
        stdDevImu.e_of_x = [0.0 for i in range(3)]
        stdDevImu.e_of_x_squ = [0.0 for i in range(3)]
        stdDevImu.stdDev = [0.0 for i in range(3)]
        stdDevImu.buffer_index = 0
        stdDevImu.data_filled = False
        stdDevImu_init = True
        stdDevImu.diff_count = 0
        stdDevImu.init_data = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        print 'start, %.32f, %.32f, %.32f, %.32F' %(orientation.x, orientation.y, orientation.z, orientation.w)
    if stdDevImu.data_filled == False:
        euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        if stdDevImu.init_data[0] != euler[0] or stdDevImu.init_data[1] != euler[1] or stdDevImu.init_data[2] != euler[2]:
            stdDevImu.diff_count += 1
            print stdDevImu.diff_count
            print "1"
            print 'start, %.32f, %.32f, %.32f, %.32F' %(orientation.x, orientation.y, orientation.z, orientation.w)
            print 'start, %.32f, %.32f, %.32f' %(euler[0], euler[1], euler[2])            
        # # fill in data
        stdDevImu.input_data[0][stdDevImu.buffer_index] = euler[0]
        stdDevImu.input_data[1][stdDevImu.buffer_index] = euler[1]
        stdDevImu.input_data[2][stdDevImu.buffer_index] = euler[2]
        # # print 'transformed, %.32f, %.32f, %.32f' %( euler[0], euler[1], euler[2])
        stdDevImu.buffer_index += 1
        if stdDevImu.buffer_index < INPUT_SIZE:
            return (False, stdDevImu.stdDev)
        else:
            stdDevImu.data_filled = True
            for i in range(3):
                for j in range(INPUT_SIZE):
                    stdDevImu.e_of_x[i] += stdDevImu.input_data[i][j]
                    stdDevImu.e_of_x_squ[i] += stdDevImu.input_data[i][j] * stdDevImu.input_data[i][j]
            print stdDevImu.e_of_x, stdDevImu.e_of_x_squ
            for i in range(3):
                stdDevImu.stdDev[i] = math.sqrt(stdDevImu.e_of_x_squ[i] / INPUT_SIZE - math.pow((stdDevImu.e_of_x[i] / INPUT_SIZE), 2))
            print 'end, %.32f, %.32f, %.32f, %.32F' %(orientation.x, orientation.y, orientation.z, orientation.w)
            return (True, stdDevImu.stdDev)
    else:
        return (True, stdDevImu.stdDev)



def readIMU():

    full_turns = 0
    prev_heading, _, _ = bno.read_euler()
    init_heading = prev_heading
    global stdDevImu_init
    stdDevImu_init = False
    #headings = Float32MultiArray()
    data = BOSCH_IMU_DATA() 
    imu_msg = Imu()
    imu_msg.header.frame_id = "docking_base_link"
    imu_msg.angular_velocity_covariance[0] = 1 
    imu_msg.angular_velocity_covariance[4] = 1
    imu_msg.angular_velocity_covariance[8] = 1
    imu_msg.linear_acceleration_covariance[0] = 1
    imu_msg.linear_acceleration_covariance[4] = 1
    imu_msg.linear_acceleration_covariance[8] = 1

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
        # # print 'raw_data, %.32f, %.32f, %.32f' %(roll/180*3.1415926, pitch/180*3.1415926, heading/180*3.1415926)
        current_time = rospy.get_rostime()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
#        sys, gyro, accel, mag = bno.get_calibration_status()
#        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#          heading,roll,pitch, sys, gyro, accel, mag))
        #value=mymessages()

        if (prev_heading > 270) and (heading < 90):
            full_turns = full_turns + 1
        if (prev_heading < 90) and (heading > 270):
            full_turns = full_turns - 1
        prev_heading = heading
        heading = full_turns * 360 + heading
	heading = heading - init_heading
        #headings.data = [full_turns, heading]
        data.full_turns = full_turns
        data.euler_angles.z, data.euler_angles.y, data.euler_angles.x = heading, pitch, roll
        quaternions = tf.transformations.quaternion_from_euler(roll/180*3.1415926,pitch/180*3.1415926,heading/180*3.1415926)
        imu_msg.header.stamp = current_time
        imu_msg.orientation.x = quaternions[0]
        imu_msg.orientation.y = quaternions[1]
        imu_msg.orientation.z = quaternions[2]
        imu_msg.orientation.w = quaternions[3]
        # (std_dev_valid, std_dev) = stdDevImu(imu_msg.orientation)

        imu_msg.angular_velocity.x = data.angular_velocity.x
        imu_msg.angular_velocity.y = data.angular_velocity.y
        imu_msg.angular_velocity.z = data.angular_velocity.z
        imu_msg.linear_acceleration.x = data.linear_acceleration.x
        imu_msg.linear_acceleration.y = data.linear_acceleration.y
        imu_msg.linear_acceleration.z = data.linear_acceleration.z
        imu_msg.orientation_covariance = [0.0005552915788564602, 0.0, 0.0, 0.0, 0.00098195341254686, 0.0, 0.0, 0.0, 0.00007985410657169154]

        # if std_dev_valid == True:
        #     imu_msg.orientation_covariance[0] = std_dev[0]
        #     imu_msg.orientation_covariance[4] = std_dev[1]
        #     imu_msg.orientation_covariance[8] = std_dev[2]
        #     pub.publish(imu_msg)
        pub.publish(imu_msg)
        pub_theta.publish(heading)
            #pub_gyrospeed.publish(z)
        pub_rotspeed.publish(data.angular_velocity.z)
        pub_imu.publish(data)
        rate.sleep()


if __name__ == '__main__':
    readIMU()
