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
import logging
import sys
import time
import rospy
import signal
import math
from std_msgs.msg import Float64

from Adafruit_BNO055 import BNO055


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
#calibration_values=[235, 255, 170, 255, 4, 0, 2, 255, 68, 0, 13, 255, 0, 0, 255, 255, 255, 255, 232, 3, 159, 2]
#bno.set_calibration(calibration_values)

pub_vel= rospy.Publisher('IMU_Vx', Float64, queue_size=1)
pub_disp = rospy.Publisher('IMU_DISPLx', Float64, queue_size=1)
pub_acc = rospy.Publisher('IMU_Acx', Float64, queue_size=1)
rospy.init_node('IMU_integration', anonymous=False)
rate = rospy.Rate(100)

def readIMU():
	accelold=0
	velold=0
	displold=0
	while not rospy.is_shutdown():
 		# Read the Euler angles for heading, roll, pitch (all in degrees).
    		heading, roll, pitch = bno.read_euler()
    		# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    		sys, gyro, accel, mag = bno.get_calibration_status()
		
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
    		wx,wy,wz = bno.read_gyroscope()
    		# Accelerometer data (in meters per second squared):
    		#x,y,z = bno.read_accelerometer()
    		# Linear acceleration data (i.e. acceleration from movement, not gravity--
    		# returned in meters per second squared):
    		Acx,Acy,Acz = bno.read_linear_acceleration()
    		newvelx=velold+(((accelold+Acx)/2.0)*0.01)
    		newdisplx=displold+(((velold+newvelx)/2.0)*0.01)
   		# print ('Xddot={0:0.2F} Yddot={1:0.2F} Zddot={2:0.2F}\tHeading={3:0.2F} Roll={4:0.2F} Pitch={5:0.2F}\tSys_cal={6} Gyro_cal={7} Accel_cal={8} Mag_cal={9}'.format(z,z,z,heading,roll,pitch, sys, gyro, accel, mag))
#    		print ("acc: ",Acx)
#		print ("vel: ",newvelx)
 #   		print ("displ: ",newdisplx) 
    		velold=newvelx
    		displold=newdisplx
    		accelold=Acx
		
		pub_acc.publish(Acx)
		pub_vel.publish(newvelx)
		pub_disp.publish(newdisplx)
		# Gravity acceleration data (i.e. acceleration just from gravity--returned
		# in meters per second squared):
    		#x,y,z = bno.read_gravity()
    		# Sleep for a second until the next reading.
    		rate.sleep()

if __name__ == '__main__':
	readIMU()	
