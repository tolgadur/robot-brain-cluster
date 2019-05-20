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
ext_ref = 0

def Get_IMU_Data(msg):
	global current_theta
	current_theta = msg.euler_angles.z

def Get_Reference(msg):
	global theta_ref
	ext_ref = msg.data
	theta_ref = current_theta + ext_ref	#user must make sure reference is only sent once so that setpoint(theta_ref) is not continuously increasing

rospy.init_node('Reference_publisher', anonymous=False)
sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_IMU_Data)
sub_ref = rospy.Subscriber('Theta_Ref' Float64, Get_Reference)
pub_ref = rospy.Publisher('ref_theta', Float64, queue_size=1, latch=True)	#setpoint published to controller
rate = rospy.Rate(10)

if __name__ == '__main__':
    try:
	rospy.sleep(0.5)	# Short delay so that current heading(IMU) is read before publishing reference at initialisation
	theta_ref = current_theta
	pub_ref.publish(theta_ref)
        while not rospy.is_shutdown():
            #theta_ref = current_theta + ext_ref		#ext_ref is added in each loop
	    print('Publishing : % s'% (theta_ref))
            pub_ref.publish(theta_ref)
            rate.sleep()
    except rospy.ROSInterruptException:
            pass
