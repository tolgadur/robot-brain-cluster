#!/usr/bin/env python
# Simple Adafruit BNO055 sensor reading example.  Will print the orientation
# and calibration data every second.
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import rospy
import logging
import sys
import time
import math
import os
import signal
import serial
import xbox
from std_msgs.msg import Float32
#from imu_emilk.msg import mymessages
from Adafruit_BNO055 import BNO055

integral=0
derivative=0
preverror=0
kp=1
ki=0
kd=1
setpoint=0
SPEED_MULTIPLIER = 1.0          # Speed scaling factor

   
joy         = xbox.Joystick()           # Initialize joystick

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=18)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
a,b,c,d,e,f=bno.get_axis_remap()
print('Xaxis={0:0} Yaxis={1:0} Zaxis={2:0} Xsign={3:0} Ysign={4:0} Zsign={5:0}'.format(a,b,c,d,e,f))

pub_theta = rospy.Publisher('theta_gyro', Float32, queue_size=1)
pub_gyrospeed = rospy.Publisher('angular_speed', Float32, queue_size=1)
pub_omega = rospy.Publisher('speed_command', Float32, queue_size=1)
pub_accel= rospy.Publisher('x_accelerometer_data', Float32, queue_size=1)
pub_linvelocity= rospy.Publisher('linear_velocity', Float32, queue_size=1) 
rospy.init_node('IMU_control', anonymous=False)
rate = rospy.Rate(10)   # 10hz sampling/publishing frequency 



def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    joy.close()
    sys.exit(0)


def waitConnect():
    # Waiting for joystick connection
    print("-- Waiting for joystick connection")
    while not joy.connected():
        time.sleep(0.1)
    print("-- Joystick connected")

    
def readIMU():
    global integral
    global derivative
    global preverror
    #global myarray
    deadzone=5  #degrees
    while not rospy.is_shutdown():      # The loop samples, CALCULATES AND PUBLISHES
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        #heading, roll, pitch = bno.read_euler()
	# Linear acceleration data (i.e. acceleration from movement, not gravity--
	# returned in meters per second squared):
	xddot,yddot,zddot = bno.read_linear_acceleration()
	# Gyroscope data (in degrees per second):
	x,y,z = bno.read_gyroscope()
        # Gyroscope data (in degrees per second):
        heading,roll,pitch= bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
        #  heading,roll,pitch, sys, gyro, accel, mag))
	#value=mymessages()
        #----------------CONTROLLER BEGINS HERE -------------------------------------
	if heading>180:
	    heading=heading-360 	#Gets you negative angles 
        error=setpoint-heading
        #derivative=erroradj-preverror
	print('Heading={0:0.2F} error={1:0.2F} derivative={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading,error,derivative, sys, gyro, accel, mag))
	if error>deadzone:		#SUBSTRACT DEADZONE in order to compensate for
	    erroradj=error-deadzone	# SLOW RESPONSE (HIGH TIME CONSTANT)
	elif error<-deadzone:
	    erroradj=error+deadzone
	else:
	    erroradj=0
        derivative=erroradj-preverror 
	if erroradj>0:
            speed=((kp*erroradj)+(kd*derivative)+10.99)  #THE + is probably a -,check the value with the joystick. (in the 2 lines)
        else:
            speed=((kp*erroradj)+(kd*derivative)-10.99)
	preverror=erroradj	#cual es tu mejor choice para definir error?(afect derivative) el erroradj or el true error
        #--------CONTROLLER ENDS HERE-----------------------------------------
        if not joy.connected():
            print("-- Error! Joystick disconnected - waiting for connection")
            lspeed      = 0

        else:
            print("Joystick Connected")  
            left_trig   = joy.leftTrigger()
            right_trig  = joy.rightTrigger()
            lspeed      = 100*(right_trig) - 100*(left_trig)
            lspeed      = lspeed*SPEED_MULTIPLIER
        #myarray.append(heading)
	#plt.plot(myarray)
	#value.gyro_theta=heading
	#value.gyro_speed=z
	#value.angular_velocity=speed
	#value.accelerometer_data=xddot
	#value.linear_velocity=14.0
	#rospy.loginfo(value)     
        #pub.publish(value)
        pub_theta.publish(heading)
	pub_gyrospeed.publish(z)
	pub_omega.publish(speed)
	pub_accel.publish(xddot)
	pub_linvelocity.publish(lspeed)
	rate.sleep()


if __name__ == '__main__':
    waitConnect()
    #try:
    #    readIMU()
        #Controloop()
    #except rospy.on_shutdown(cleanupOnExit)
    readIMU()
    rospy.on_shutdown(cleanupOnExit)
    
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    #heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    #sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          #heading, roll, pitch, sys, gyro, accel, mag))
    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
	#temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
    #x,y,z = bno.read_gyroscope()
    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    #print ('Xddot={0:0.2F} Yddot={1:0.2F} Zddot={2:0.2F}\tHeading={3:0.2F} Roll={4:0.2F} Pitch={5:0.2F}\tSys_cal={6} Gyro_cal={7} Accel_cal={8} Mag_cal={9}'.format(z,z,z,heading, roll,pitch, sys, gyro,accel,mag)) 
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    #time.sleep(0.5)

