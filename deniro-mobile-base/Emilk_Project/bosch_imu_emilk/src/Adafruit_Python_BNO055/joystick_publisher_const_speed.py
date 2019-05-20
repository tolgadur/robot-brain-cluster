#!/usr/bin/env python

import rospy
import logging
import sys
import time
import math
import os
import signal
import xbox
from std_msgs.msg import Float64
#from imu_emilk.msg import mymessages

SPEED_MULTIPLIER = 1.0          # Speed scaling factor
joy         = xbox.Joystick()           # Initialize joystick
V=40

# INITIALIZE PUBLISHERS
pub_LinSpeed = rospy.Publisher('linear_speed', Float64, queue_size=1)
rospy.init_node('Joystick_speed', anonymous=False)
rate = rospy.Rate(100)   # 10hz sampling/publishing frequency 


def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    pub_LinSpeed.publish(0.0)	# Clear/stop base on exit
    joy.close()
    time.sleep(0.1)
    sys.exit(0)


def waitConnect():
    # Waiting for joystick connection
    print("-- Waiting for joystick connection")
    while not joy.connected():
        time.sleep(0.1)
    print("-- Joystick connected")


def readJoystick():
    lin_speed   = 0
    while not rospy.is_shutdown():      # The loop published commnands for linear speed from Xbox Joystick
        
        if not joy.connected():
            print("-- Error! Joystick disconnected - waiting for connection")
            lin_speed = 0

        else:
            print("Joystick Connected")
            left_trig   = joy.leftTrigger()
            right_trig  = joy.rightTrigger()
            lin_speed   = 100*(right_trig) - 100*(left_trig)
	    if lin_speed >= V:
		lin_speed = V
	    elif lin_speed <=-V:
		lin_speed = -V

	pub_LinSpeed.publish(lin_speed)
        rate.sleep()


if __name__ == '__main__':
    waitConnect()
    rospy.on_shutdown(cleanupOnExit)
    readJoystick()
    
