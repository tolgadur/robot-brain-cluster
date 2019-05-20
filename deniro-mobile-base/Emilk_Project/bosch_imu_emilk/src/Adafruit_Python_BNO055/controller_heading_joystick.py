#!/usr/bin/env python

import rospy
import logging
import sys
import time
import math
import os
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from bosch_imu_emilk.msg import BOSCH_IMU_DATA
#from imu_emilk.msg import mymessages

integral=0
derivative=0
preverror=0
kp=2
ki=0
kd=200

global theta, theta_ref, lin_speed
theta = 0
theta_ref = 0
lin_speed = 0

def Get_IMU_Data(data):
    global theta
    theta = data.euler_angles.z

def Get_Reference(msg):
    global theta_ref
    theta_ref = msg.data

def Get_Linear_Velocity(msg):
    global lin_speed 
    lin_speed = msg.data

def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    time.sleep(0.5)
    clearData = Float32MultiArray()
    clearData.data = [0.0,0.0]
    pub_speeds.publish(clearData)   # Publishing 0 values on topic
    time.sleep(1.5) # give time for the publisher to send the message
    sys.exit(0)


def control():
    global integral
    global derivative
    global preverror
    global theta, theta_ref, lin_speed
    deadzone = 0.01  # +/- degrees (double)
    min_speed = 10.99 # the minimum required ref. speed for the mobile base to start moving
    while not rospy.is_shutdown():      
        #----------------CONTROLLER BEGINS HERE -------------------------------------
        setpoint = theta_ref
	error = setpoint - theta	# Actual error
        #derivative=erroradj-preverror
        print('Heading={0:0.2F} error={1:0.2F} derivative={2:0.2F}'.format(theta,error,derivative))
        if error > deadzone:              #SUBSTRACT DEADZONE in order to compensate for
            erroradj = error - deadzone     # SLOW RESPONSE (HIGH TIME CONSTANT)
        elif error < -deadzone:
            erroradj = error + deadzone
        else:
            erroradj=0
        derivative=erroradj-preverror
        if erroradj>0:
            rot_speed=((kp*erroradj)+(kd*derivative) + min_speed)  # turning right is positive value
        else:
            rot_speed=((kp*erroradj)+(kd*derivative) - min_speed)
        preverror=erroradj      
        #--------CONTROLLER ENDS HERE-----------------------------------------
        
	# BUILD ARRAY message        
	speedmsg = Float32MultiArray()
	speedmsg.data = [lin_speed,rot_speed]

	pub_speeds.publish(speedmsg)
	pub_error.publish(erroradj)
        rate.sleep()

# INITIALIZE PUBLISHERS & SUBSCRIBERS
#sub_theta = rospy.Subscriber('theta_gyro', Float32MultiArray, Get_IMU_Data) # from IMU
sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, Get_IMU_Data)
sub_theta_ref = rospy.Subscriber('ref_theta', Float32, Get_Reference) # from kbd or Xbox controller
sub_joystick = rospy.Subscriber('linear_speed', Float32, Get_Linear_Velocity)
pub_speeds = rospy.Publisher('speed_commands', Float32MultiArray, queue_size=1)
pub_error = rospy.Publisher('error', Float32, queue_size=1)
rospy.init_node('Controller_heading', anonymous=False)
rate = rospy.Rate(100)   # 100 Hz sampling/publishing frequency 


if __name__ == '__main__':
    rospy.on_shutdown(cleanupOnExit)
    control()
    rospy.spin()
