#!/usr/bin/env python
"""
Interpreter Client.

File name: interpreter_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
import sys
from interpreter.srv import Interpreter

class InterpreterClient():

    def make_request(self, voice_input, lang_code="en-EN"):
        """
        Makes service request.

        Keyword arguments:
        voice_input -- string
        lang_code -- string
        """

        rospy.wait_for_service('interpreter')
        try:

            print("trying")
            interpreter = rospy.ServiceProxy('interpreter', Interpreter)
            #return (-8, 5, [], [], ['baseball glove', 'baseball bat', 'sports ball'])
            response = interpreter(voice_input, lang_code)
            #rospy.set_param(INPUT, response.action)
            print "Client detected: Interpreter"
            print(response.instruction_id)
            print(response.faces)
            print(response.objects)
            print("trying to return")
            return response.instruction_id, response.faces, response.objects, response.conflicts
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
