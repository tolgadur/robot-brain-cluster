#!/usr/bin/env python

import rospy
import rospy
import logging
import sys
import time
import os
import signal
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from bosch_imu_emilk.msg import BOSCH_IMU_DATA
from bosch_imu_emilk.srv import TurningRequest

# current_theta = 0	# Initialisation of variable

class RotationService:
    def __init__(self, theta=0):
        self.current_theta = theta
        self.sub_imu = rospy.Subscriber('IMU_data', BOSCH_IMU_DATA, self.Get_IMU_Data)
        self.pub_ref = rospy.Publisher('ref_theta', Float64, queue_size=1, latch=True)
        self.pub_ref.publish(self.current_theta)
        self.rate = rospy.Rate(10)

    def Get_IMU_Data(self, msg):
    	self.current_theta = msg.euler_angles.z

    def turn(self, request):
        print(request.direction)
        if request.direction == 'right':
            self.current_theta += request.angle
        else:
            self.current_theta -= request.angle

        self.pub_ref.publish(self.current_theta)

        return 0


if __name__ == "__main__":
    rospy.init_node('Reference_publisher', anonymous=False)
    rotation_service = RotationService()
    s = rospy.Service('turning', TurningRequest, rotation_service.turn)
    rospy.spin()
