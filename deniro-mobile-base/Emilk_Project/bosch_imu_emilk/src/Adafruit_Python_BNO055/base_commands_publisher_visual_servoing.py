#!/usr/bin/env python

import rospy
import logging
import sys
import time
import math
import os
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from bosch_imu_emilk.msg import BOSCH_IMU_DATA
#from imu_emilk.msg import mymessages

rot_speed = 0
lin_speed = 0
w_limit = 15

def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    clearData = Float32MultiArray()
    clearData.data = [int(0),int(0)]
    pub_speeds.publish(clearData)   # Publishing 0 values on topic
    time.sleep(2) # give time for the publisher to send the message
    sys.exit(0)

def Get_angularspeed(data):
    global rot_speed
    rot_speed = data.data	# Get control signal (angular velocity command) from PID node
    #if rot_speed > 0:
	#rot_speed = rot_speed + 10.99
    #elif rot_speed < 0:
	#rot_speed = rot_speed - 10.99

    if rot_speed >w_limit:
	rot_speed=w_limit
    elif rot_speed <-w_limit:
	rot_speed=-w_limit

def Get_linearspeed(msg):
    global lin_speed
    lin_speed = msg.data

def attachCommands():
    while not rospy.is_shutdown():
    	# BUILD ARRAY message        
    	speedmsg = Float32MultiArray()
    	speedmsg.data = [lin_speed,rot_speed]

    	pub_speeds.publish(speedmsg)	# Sends speed commands to ros2mbedserial node
    	rate.sleep()

rospy.init_node('base_commands', anonymous = False)
subs_omega = rospy.Subscriber('angular_speed', Float64, Get_angularspeed)
subs_linspeed = rospy.Subscriber('linear_speed', Float64, Get_linearspeed)
pub_speeds = rospy.Publisher('speed_commands', Float32MultiArray, queue_size=1)
rate = rospy.Rate(100)

if __name__ == '__main__':
	rospy.on_shutdown(cleanupOnExit)
	attachCommands()
	
