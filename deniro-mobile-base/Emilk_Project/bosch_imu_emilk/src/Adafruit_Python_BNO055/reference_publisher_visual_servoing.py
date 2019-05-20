#!/usr/bin/env python

import rospy
import logging
import sys
import time
import os
import signal
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from bosch_imu_emilk.msg import BOSCH_IMU_DATA

current_theta = 0	# Initialisation of variable
theta_ref = 0

def Get_IMU_Data(msg):
	global current_theta
	current_theta = msg.data

def Get_ref(data):
	global theta_ref
	theta_ref = data.data

rospy.init_node('Reference_publisher', anonymous=False)
sub_imu = rospy.Subscriber('state', Float64, Get_IMU_Data)
sub_ref = rospy.Subscriber('theta_ref', Float64, Get_ref)
pub_ref = rospy.Publisher('setpoint', Float64, queue_size = 1, latch=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    try:
	rospy.sleep(0.5)
	setpoint = current_theta
	pub_ref.publish(setpoint)
        while not rospy.is_shutdown():
            #kb_theta = float(input("Enter desired heading : "))
            #theta_ref = current_theta + kb_theta
	    setpoint = current_theta + theta_ref
	    print('Publishing : % s'% (setpoint))
            pub_ref.publish(setpoint)
            rate.sleep()
    except rospy.ROSInterruptException:
            pass
