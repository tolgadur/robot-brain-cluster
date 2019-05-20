#!/usr/bin/evn python

import rospy
import sys
import time
import os
from std_msgs.msg import Float64
from bosch_imu_emilk.msg import BOSCH_IMU_DATA

raw_accel_x = 0
filtered_accel_x = 0

alpha = 0 	# to be set according to the desired cutoff frequency

def Get_IMU_Data(msg):
	global accel_x
    raw_accel_x = msg.linear_acceleration.x

def filter():
	global filtered_accel_x
	while not rospy.is_shutdown():
		filtered_accel_x = alpha * raw_accel_x + (1 - alpha) * filtered_accel_x		# Digital implementation of 1st order RC LPF (difference equation)
		pub.publish(filtered_accel_x)
		print('Filtered data: %s' % (filtered_accel_x))	# for debug
		rate.sleep()

rospy.init_node('Imu_LPF', anonymous=False)
sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_IMU_Data)
pub = rospy.Publisher('Filter_data', Float64, queue_size=1)
rate = rospy.Rate(100)   # 100 Hz sampling/publishing frequency 

if __name__ == '__main__':
	filter()
