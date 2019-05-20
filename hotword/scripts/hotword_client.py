#!/usr/bin/env python

"""
Hotword Client.

File name: hotword_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import sys
import rospy
from hotword.srv import LangCode

LANGUAGE = 'language'

class HotwordClient():

    def make_request(self, request = None):
        """
        Makes service request.

        Keyword arguments:
        request -- string (lang_code)
        """

        try:
            rospy.wait_for_service('hotword_recognition', timeout=3)
            recognizer = rospy.ServiceProxy('hotword_recognition', LangCode)
            response = recognizer(request)
            print "Client detected: " + str(response.language_code)
            return response.language_code
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e
