#!/usr/bin/env python
"""
Interpreter Service.

File name: interpreter_service.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import sys
import os
import rospy
import interpreter_class_new as itpr

from interpreter.srv import Interpreter

class InterpreterService():
    """
    Hotword Recognition Service

    Keyword arguments:
    """
    def translate_string(self, request):
        # return [['2', '-1', 'person']]

        interpreter = itpr.SpeechInterpreter(request.voice_input, request.lang_code)
        code = interpreter.consolidate_input()

        if code < 0 and code > -99:
            print "Error code: " + str(code)
            return code,[],[],[]

        array = interpreter.get_msg()
        instruction_id = array[0]
        faces = array[1]
        objects = array[2]
        conflicts = array[3]

        if len(instruction_id) == 0:
            i_d = -1
        else:
            i_d = int(instruction_id[0])

        print "Returning: " + str(code) + " " + str(i_d)
        print(faces)
        print(objects)
        print(conflicts)
        return int(i_d), faces, objects, conflicts

if __name__ == "__main__": # pragma: no cover
    print "Speech Interpretation"
    rospy.init_node('interpreter_server')
    interpreter_server = InterpreterService()
    s = rospy.Service('interpreter', Interpreter, interpreter_server.translate_string)
    rospy.spin()
