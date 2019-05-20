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
imu_data_received = False

def Get_IMU_Data(msg):
    global current_theta, imu_data_received
    current_theta = msg.data  #euler_angles.z
    imu_data_received = True

rospy.init_node('Reference_publisher', anonymous=False)
sub_imu = rospy.Subscriber('imu_heading_rel', Float64, Get_IMU_Data)
pub_ref = rospy.Publisher('ref_theta', Float64, queue_size=1, latch=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    #try:
    #rospy.sleep(5)
    print('Waiting for IMU data to arrive on topic imu_heading_rel')
    while not imu_data_received:
        rospy.sleep(0.1)
    print('Received IMU data, current heading = ', current_theta)

    theta_ref = current_theta
    pub_ref.publish(theta_ref) # why?
    while not rospy.is_shutdown():
    	kb_theta = float(input("Enter new desired relative heading in degrees: "))
    	theta_ref = current_theta + kb_theta
        print('Publishing ref_theta: % s'% (theta_ref))
    	pub_ref.publish(theta_ref)
    	rate.sleep() # why?
#    except rospy.ROSInterruptException:
#        pass
