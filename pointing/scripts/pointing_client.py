#!/usr/bin/env python
"""
Pointing Client.

File name: pointing_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
import sys
from pointing.srv import PointingRequest
from pointing_translation import PointingTranslation

class PointingClient():

    def make_request(self, filename):
        """
        Makes service request.

        Keyword arguments:
        filename -- string
        """

        try:
            rospy.wait_for_service('pointing', timeout=3)
            pointer = rospy.ServiceProxy('pointing', PointingRequest)
            pointer(filename)
            # return response.grasping_status
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    client = PointingClient()
    client.make_request('/home/petar/brain-cluster-ws/src/state_machine/src/resources/joint_position_right.txt')
