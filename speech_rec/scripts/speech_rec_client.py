#!/usr/bin/env python
"""
Speech Rec Client.

File name: speech_rec_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
import sys
from speech_rec.srv import GetSentence

SENTENCE = 'language'

class SpeechRecClient():

    def make_request(self, lang_code):
        """
        Makes service request.

        Keyword arguments:
        lang_code -- string
        """

        try:

            rospy.wait_for_service('speech_recognition', timeout=3)
            speech_recognizer = rospy.ServiceProxy('speech_recognition', GetSentence)
            response = speech_recognizer(lang_code)
            print "Speech Rec client detected: " + str(response.sentence)
            return response.sentence
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e
