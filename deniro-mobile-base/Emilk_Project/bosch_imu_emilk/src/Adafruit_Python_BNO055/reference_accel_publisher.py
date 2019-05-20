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

#current_theta = 0       # Initialisation of variable

rospy.init_node('Reference_publisher', anonymous=False)
#sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_IMU_Data)
pub_ref = rospy.Publisher('ref_accel_X', Float64, queue_size=1, latch=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    try:
        #rospy.sleep(0.5)
        #theta_ref = current_theta
        #pub_ref.publish(theta_ref)
	accel_ref = 0
	pub_ref.publish(accel_ref)
        while not rospy.is_shutdown():
            kb_accelX = float(input("Enter desired acceleration : "))
            #accel_ref = accel_ref + kb_accelX 
            accel_ref = kb_accelX
	    print('Publishing : % s'% (accel_ref))
            pub_ref.publish(accel_ref)
            rate.sleep()
    except rospy.ROSInterruptException:
	    #pub_ref.publish(0.0)
            pass

