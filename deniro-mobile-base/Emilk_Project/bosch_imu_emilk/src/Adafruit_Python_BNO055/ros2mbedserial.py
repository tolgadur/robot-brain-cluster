#!/usr/bin/env python

import rospy
import math
import os
import time
import signal
import serial
import sys
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Int16
from array import array

# Protocol Data
HEADER      = 127       # Magic sync Header for the serial packet communication with the mbed
SPEED_CMD   = int(83)
MODE_CMD    = int(73)
RESET_CMD   = int(63)
INIT_CMD    = int(53)

# Serial port data
PORT_NAME = "/dev/ttyUSB0"
# BAUD_RATE = int(230400)
BAUD_RATE = int(115200)

serial_port = serial.Serial()


def initSerial():
  serial_port.baudrate = BAUD_RATE
  serial_port.port     = PORT_NAME
  serial_port.open()


def packData(cmd, velocity, direction, lights):
  data = bytearray()
  data.append(HEADER)
  data.append(cmd)
  data.append(velocity & 0xff)
  data.append(direction & 0xff)
  data.append(lights & 0xff)
  return data

def cleanupOnExit():
  print("-- Cleaning up and quiting!")
  time.sleep(1.0)	# Delay just in case the program is closed when sending a value to mbed
  serial_port.write(packData(SPEED_CMD, int(0), int(0), int(0)))
  time.sleep(1.0)
  print("-- Mobile base stopped")

  serial_port.close()
  sys.exit(0)


def sendData(msg):
  #global v,w
  speed       = int(msg.data[0])	#linear velocity
  direction   = int(msg.data[1])	#angular velocity
  #cmd = SPEED_CMD			# The next two lines are commented to save time and avoid timeout in mbed
  #print("Sendind data -- Linear Speed: %d Angular Speed: %d" % (speed, direction))
  if direction > 100:
 	direction = 100
  elif direction < -100:
  	direction = -100
  data = packData(SPEED_CMD, speed, direction, int(0))
  serial_port.write(data)


def readData():
  pack_start = False
  int_data = [0,0,0] # initialise int_data array
  raw_data = [0,0,0]
  while(not rospy.is_shutdown()):
    while((not pack_start) and (not rospy.is_shutdown())):
      this_raw_data = serial_port.read(1)  # get raw data (1 byte) from serial port
      this_int_data = ord(this_raw_data) # convert string to 8 bit int
      if this_int_data == 127:  # check if raw data is start of pack
        pack_start = True # set indicater
    for j in range(3):
      raw_data[j] = serial_port.read(1) # get raw data (3 bytes) from serial port
      int_data[j] = ord(raw_data[j])
    battery_voltage = Int16()
    battery_voltage.data = int_data[0]*256+ int_data[1] #convert two 8 bit int back to single 16 bit int
    pack_start = False
    timeout = Bool()
    timeout.data = bool(int_data[2])
    battery_volt_pub.publish(battery_voltage) # publish battery_voltage


if __name__ == '__main__':
    try:
        # Handle kill commands from terminal
        signal.signal(signal.SIGTERM, cleanupOnExit)

        # initialize the serial port connection
        initSerial()
        rospy.on_shutdown(cleanupOnExit)
        print("--Mobile base node running--")
        rospy.init_node('base_node', anonymous=False)
        rospy.Subscriber('speed_commands', Float32MultiArray, sendData)
        battery_volt_pub = rospy.Publisher('battery_voltage', Int16, queue_size = 2)
        data = packData(INIT_CMD,0,0,0)         # Send initialisation command to mbed to reset the timeout
        serial_port.write(data)
        readData()

    except rospy.ROSInterruptException:
        cleanupOnExit()
#	pass
