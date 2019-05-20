#!/urs/bin/env python

import rospy
import math
import os
import time
import signal
import serial
import sys
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray

# Protocol Data
HEADER      = 127       # Magic sync Header for the serial packet communication with the mbed
SPEED_CMD   = int(83)
MODE_CMD    = int(73)
RESET_CMD   = int(63)

global speed, direction, cmd
speed = 0
direction = 0
cmd = SPEED_CMD

# Serial port data
PORT_NAME = "/dev/ttyUSB0"
# BAUD_RATE = int(230400)
BAUD_RATE = int(115200)

serial_port = serial.Serial()


def initSerial():
    serial_port.baudrate = BAUD_RATE
    serial_port.port     = PORT_NAME
    serial_port.open()


def packData(velocity, direction, lights, cmd=None):
    cmd = CMD if cmd is None else cmd
    data = bytearray()
    data.append(HEADER)
    data.append(cmd)
    data.append(velocity & 0xff)
    data.append(direction & 0xff)
    data.append(lights & 0xff)

    return data

def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    serial_port.write(packData(SPEED_CMD, int(0), int(0), int(0)))
    print("-- Mobile base stopped")

    serial_port.close()
    #joy.close()
    sys.exit(0)

#def getAngularVel(data):
#    global w
#    rospy.loginfo(rospy.get_caller_id() + "Callback1 heard: %s", data.data)
#    w = data.data

#def getLinearVel(data):
#    global v
#    rospy.loginfo(rospy.get_caller_id() + "Callbak2 heard: %s", data.data)
#    v = data.data
def getSpeeds(msg):
	global speed, direction
	speed		= int(msg.data[0])	#Linear Velocity
	direction 	= int(msg.data[1])	#Angular Velocity
	#print("Sendind data -- Linear Speed: %d Angular Speed: %d" % (speed, direction))


def ResetJoy(sig):
	global cmd
	if sig.data == 1:
		cmd = RESET_CMD
	else:
		cmd = SPEED_CMD

def sendData():
    global speed, direction, cmd
    print("Sendind data -- Linear Speed: %d Angular Speed: %d" % (speed, direction))
    data = packData(cmd, speed, direction, int(0))
    serial_port.write(data)


def movebase():
        print("--Mobile base node running--")
	rospy.init_node('base_node', anonymous=False)
	#rospy.Subscriber('speed_command',Float32, getAngularVel)
	#rospy.Subscriber('linear_velocity',Float32, getLinearVel)
	rospy.Subscriber('speed_commands', Float32MultiArray, getSpeeds)
	rospy.Subscriber('reset_command', Int8, ResetJoy)
	#sendData()
	#rospy.spin()

if __name__ == '__main__':
    try:
        # Handle kill commands from terminal
        signal.signal(signal.SIGTERM, cleanupOnExit)

        # initialize the serial port connection
        initSerial()

        movebase()
	while not rospy.is_shutdown():
		sendData()

	rospy.spin()
	rospy.on_shutdown(cleanupOnExit)
    except rospy.ROSInterruptException:
        cleanupOnExit()
