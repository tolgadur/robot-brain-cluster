#!/usr/bin/env python

import rospy
import logging
import sys
import time
import math
import os
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
#from imu_emilk.msg import mymessages

integral=0
derivative=0
preverror=0
kp=3
ki=0
kd=30

global theta, theta_ref
theta = 0
theta_ref = 0

def Get_IMU_Data(data):
    global theta
    theta = data.data[0]

def Get_Reference(msg):
    global theta_ref
    theta_ref = msg.data

def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    clearData = Float32MultiArray()
    clearData.data = [int(0),int(0)]
    pub_speeds.publish(clearData)   # Publishing 0 values on topic
    time.sleep(2) # give time for the publisher to send the message
    sys.exit(0)


def control():
    global integral
    global derivative
    global preverror
    global theta, theta_ref, init_heading
    deadzone = 0.5  # +/- degrees (double)
    lin_speed   = 0    # Linear Velocity
    min_speed = 10.99 # the minimum required ref. speed for the mobile base to start moving
    while not rospy.is_shutdown():
        #----------------CONTROLLER BEGINS HERE -------------------------------------
        setpoint = theta_ref
        if (setpoint != 180 and setpoint != -180):  # Don't consider negative angles when the reference signal is 180 degrees
            if theta > 180:
                theta = theta - 360         #Gets you negative angles
	error = setpoint - theta
        #derivative=erroradj-preverror
        print('Heading={0:0.2F} error={1:0.2F} derivative={2:0.2F} Initial Heading={3:0.2F}'.format(theta,error,derivative,init_heading))
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
        preverror=erroradj      #cual es tu mejor choice para definir error?(afect derivative) el erroradj or el true error
        #--------CONTROLLER ENDS HERE-----------------------------------------

	# BUILD ARRAY message
	speedmsg = Float32MultiArray()
	speedmsg.data = [lin_speed,rot_speed]

	pub_speeds.publish(speedmsg)
        rate.sleep()

# INITIALIZE PUBLISHERS & SUBSCRIBERS
sub_theta = rospy.Subscriber('theta_gyro', Float32MultiArray, Get_IMU_Data) # from IMU
sub_theta_ref = rospy.Subscriber('ref_theta', Float64, Get_Reference) # from kbd or Xbox controller
pub_speeds = rospy.Publisher('speed_commands', Float32MultiArray, queue_size=1)
rospy.init_node('Controller_heading', anonymous=False)
rate = rospy.Rate(100)   # 100 Hz sampling/publishing frequency


if __name__ == '__main__':
    rospy.on_shutdown(cleanupOnExit)
    control()
    rospy.spin()
