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

def cleanupOnExit():
  print("-- Cleaning up and quiting!")
  clearData = Float32MultiArray()
  clearData.data = [int(0),int(0)]
  pub_speeds.publish(clearData)   # Publishing 0 values on topic
  time.sleep(0.5) # give time for the publisher to send the message
  sys.exit(0)

def getSpeed(msg):
  global lin_speed, rot_speed, subscriber_counter
  lin_speed = msg.data[0]
  rot_speed = msg.data[1]
  subscriber_counter += 1


def attachCommands():
  global lin_speed, rot_speed, subscriber_counter
  last_counter = 0
  rate = rospy.Rate(100)
  timeout_counter = 0
  while not rospy.is_shutdown():
    if subscriber_counter == last_counter:
      timeout_counter += 1
      if timeout_counter >= 50:
        print "timeout"
        lin_speed = 0
        rot_speed = 0
    else:
      timeout_counter = 0
      # BUILD ARRAY message
    speedmsg = Float32MultiArray()
    speedmsg.data = [lin_speed,rot_speed]
    pub_speeds.publish(speedmsg)    # Sends speed commands to ros2mbedserial node
    last_counter = subscriber_counter
    rate.sleep()

if __name__ == '__main__':
  global lin_speed, rot_speed, subscriber_counter
  lin_speed = 0
  rot_speed = 0
  subscriber_counter = 0
  rospy.init_node('mobilebase_open_loop_controller', anonymous = False)
  sub_speed_cmd = rospy.Subscriber('mobilebase_control_signal', Float32MultiArray, getSpeed)
  pub_speeds = rospy.Publisher('speed_commands', Float32MultiArray, queue_size=1)
  rospy.on_shutdown(cleanupOnExit)
  attachCommands()
