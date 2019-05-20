#!/usr/bin/env python

"""
Audio Out Client.

File name: audio_out_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
import sys
from audio_out.srv import PlaySentence

class AudioOutClient():

    def make_request(self, text, lang_code):
        """
        Makes service request.

        Keyword arguments:
        text -- string
        lang_code -- string
        """
        try:
            rospy.wait_for_service('audio_out', timeout=3)
            audio_out = rospy.ServiceProxy('audio_out', PlaySentence)
            response = audio_out(text, lang_code)
            print ("I said: " + str(text) + " in " + lang_code + ".")
            return response.success
        except (rospy.ServiceException, rospy.ROSException):
            print ("Service call failed: %s"%e)
