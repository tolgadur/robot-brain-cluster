#!/usr/bin/env python

import rospy
import time
import sys
import os
from std_msgs.msg import Float64
from bosch_imu_emilk.msg import BOSCH_IMU_DATA

accel_x = 0

def Get_Acceleration(msg):
    global accel_x
    accel_x = msg.linear_acceleration.x

def publishaccel():
    rospy.init_node('imu_to_acceleration', anonymous=False)
    pub_accel = rospy.Publisher('IMU_accelerationX', Float64, queue_size=1)
    sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_Acceleration)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
	pub_accel.publish(accel_x)
	rate.sleep()

if __name__ == '__main__':
    publishaccel()
