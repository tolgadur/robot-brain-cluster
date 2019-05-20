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

speed_ref = 0	# Initialisation of variable

def Get_IMU_Data(msg):
	global current_theta
	current_theta = msg.euler_angles.z

rospy.init_node('Reference_publisher', anonymous=False)
#sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_IMU_Data)
pub_ref = rospy.Publisher('ref_speed', Float64, queue_size=1, latch=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    try:
	rospy.sleep(0.2)
	#theta_ref = current_theta
	pub_ref.publish(speed_ref)
        while not rospy.is_shutdown():
            kb_speed = float(input("Enter desired heading : "))
            speed_ref = kb_speed
	    print('Publishing : % s'% (speed_ref))
            pub_ref.publish(speed_ref)
            rate.sleep()
    except rospy.ROSInterruptException:
            pass
