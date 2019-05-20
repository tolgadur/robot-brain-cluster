#!/usr/bin/env python

import rospy
import sys
from bosch_imu_emilk.srv import TurningRequest

class RotationClient():

    def make_request(self, direction, angle):
        try:
            rospy.wait_for_service('turning', timeout=3)
            turn = rospy.ServiceProxy('turning', TurningRequest)
            response = turn(direction, angle)
            print "Turning client client detected: " + str(response.turning_status)
            return response.turning_status
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e

class Angle:
    def __init__(self, direction, angle):
        self.direction = direction
        self.angle = angle

if __name__ == "__main__":

    tc = RotationClient()
    tc.make_request('right', 30)
