#!/usr/bin/env python

"""
Environment Client.

File name: environment_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import sys
import rospy
from environment.srv import Environment

class EnvironmentClient():

    def make_request(self, action_input):
        """
        Makes service request.

        Keyword arguments:
        action_input -- instruction_id
        """

        print "Environment Client reports"
        print "Action input: " + str(action_input[0])
        rospy.wait_for_service('environment_service', timeout=20)
        try:
            print(action_input[0])
            environment = rospy.ServiceProxy('environment_service', Environment)
            response = environment(action_input[0])
            #rospy.set_param(INPUT, response.action)
            print "HELLO"
            print "Client detected: " + str(response)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
